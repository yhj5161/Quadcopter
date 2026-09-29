#include "ICM42688.h"

#include "Delay.h"
#include <math.h>

/*
 * ICM42688.c — ICM-42688-P 驱动实现 + 内置姿态解算
 *
 * 通信: F407 硬件 SPI2(函数指针注入), 模式 0, ~7µs@5.25MHz
 * 采样: 500Hz 控制节拍调用 ICM42688_ReadSensor(), ICM ODR 1kHz
 * 姿态: Roll/Pitch 由加速度重力方向反算, Yaw 由 Z 轴角速度积分
 *
 * 寄存器位定义参考 Betaflight accgyro_icm42688p 驱动。
 */

/* ══════════════════════════════════════════════════════════════════════
 * 寄存器(Bank 0)
 * ══════════════════════════════════════════════════════════════════════ */
#define REG_BANK_SEL        (0x76U)
#define REG_DEVICE_CONFIG   (0x11U)
#define REG_ACCEL_DATA_X1   (0x1FU)
#define REG_PWR_MGMT0       (0x4EU)
#define REG_GYRO_CONFIG0    (0x4FU)
#define REG_ACCEL_CONFIG0   (0x50U)
#define REG_GYRO_CONFIG1    (0x51U)
#define REG_WHO_AM_I        (0x75U)

/* ══════════════════════════════════════════════════════════════════════
 * 默认配置值
 * ══════════════════════════════════════════════════════════════════════ */
#define ICM_PWR_MGMT0_LN      (0x0FU)
#define ICM_GYRO_CONFIG0      (0x08U)   /* ±2000dps */
#define ICM_ACCEL_CONFIG0     (0x08U)   /* ±16g */

/* ══════════════════════════════════════════════════════════════════════
 * 物理量换算（使用数据手册灵敏度常数）
 * ══════════════════════════════════════════════════════════════════════ */
#define GYRO_SCALE_DPS        (2000.0f / 32768.0f)     /* ±2000°/s → °/s (与 OmniM0 一致) */
#define ACCEL_SCALE_MS2       (16.0f * 9.80665f / 32768.0f)   /* ±16g → m/s²  */

/* 静止死区：角速度绝对值小于此值视为 0，抑制积分漂移 */
#define GYRO_DEADBAND_DPS     (0.25f)

/* 采样周期（秒），Yaw 积分用。SensorTask 周期 2ms。 */
#define SAMPLE_PERIOD_S       (0.002f)

/* 零偏校准样本数（上电时静止采集） */
#define BIAS_SAMPLES           200U

/* 角度换算 */
#define DEG_PER_RAD           (57.2957795f)

/* ══════════════════════════════════════════════════════════════════════
 * 状态
 * ══════════════════════════════════════════════════════════════════════ */
static const ICM42688_CtrlConfig_t *s_cfg;
static uint8_t                      s_count;

ICM42688_Data_t g_icm42688;

/* ══════════════════════════════════════════════════════════════════════
 * SPI I/O(通过函数指针注入)
 * ══════════════════════════════════════════════════════════════════════ */

static int16_t make_i16(uint8_t high, uint8_t low)
{
	return (int16_t)(((uint16_t)high << 8U) | low);
}

static void write_reg(uint8_t reg, uint8_t value)
{
	s_cfg->csFn(0);
	(void)s_cfg->transferFn((uint8_t)(reg & 0x7FU));
	(void)s_cfg->transferFn(value);
	s_cfg->csFn(1);
	Delay_us(1U);
}

static uint8_t read_reg(uint8_t reg)
{
	uint8_t value;
	s_cfg->csFn(0);
	(void)s_cfg->transferFn((uint8_t)(reg | 0x80U));
	value = s_cfg->transferFn(0xFFU);
	s_cfg->csFn(1);
	Delay_us(1U);
	return value;
}

static void read_regs(uint8_t reg, uint8_t *data, uint8_t length)
{
	s_cfg->csFn(0);
	(void)s_cfg->transferFn((uint8_t)(reg | 0x80U));
	while (length-- != 0U) { *data++ = s_cfg->transferFn(0xFFU); }
	s_cfg->csFn(1);
	Delay_us(1U);
}

/* ══════════════════════════════════════════════════════════════════════
 * 注册
 * ══════════════════════════════════════════════════════════════════════ */
void ICM42688_Register(const ICM42688_CtrlConfig_t *configTable, uint8_t count)
{
	s_cfg   = configTable;
	s_count = count;
}

/* ══════════════════════════════════════════════════════════════════════
 * ICM42688_Init — 上电初始化 + 零偏校准
 *
 * 前置: SPI initFn 注入有效, 传感器保持静止。
 * 返回: 1 成功, 0 失败。
 * ══════════════════════════════════════════════════════════════════════ */
uint8_t ICM42688_Init(void)
{
	uint8_t  retry;
	uint8_t  who_am_i = 0U;
	uint16_t sample;
	float    sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
	uint8_t  data[14];

	if ((s_cfg == 0) || (s_count == 0U)) { return 0U; }

	/* ── 注入的 SPI 初始化 ── */
	s_cfg->initFn(s_cfg->sckPort, s_cfg->sckPin,
	              s_cfg->mosiPort, s_cfg->mosiPin,
	              s_cfg->misoPort, s_cfg->misoPin,
	              s_cfg->csPort,   s_cfg->csPin);
	Delay_ms(10U);

	/* ── WHO_AM_I ── */
	for (retry = 0U; retry < 20U; retry++)
	{
		who_am_i = read_reg(REG_WHO_AM_I);
		if (who_am_i == ICM42688_WHO_AM_I_VALUE) { break; }
		Delay_ms(10U);
	}
	if (who_am_i != ICM42688_WHO_AM_I_VALUE) { return 0U; }

	/* ── 软件复位 ── */
	write_reg(REG_DEVICE_CONFIG, 0x01U);
	Delay_ms(20U);
	write_reg(REG_BANK_SEL, 0x00U);

	/* ── 量程 / 速率（复刻 OmniM0 成功参数） ── */
	write_reg(REG_GYRO_CONFIG0,  ICM_GYRO_CONFIG0);
	write_reg(REG_ACCEL_CONFIG0, ICM_ACCEL_CONFIG0);
	write_reg(REG_GYRO_CONFIG1,  0x06U);

	/* ── 低噪声模式 ── */
	write_reg(REG_PWR_MGMT0, ICM_PWR_MGMT0_LN);
	Delay_ms(50U);

	/* ── 读回配置寄存器（诊断用，验证写入是否生效） ── */
	g_icm42688.who_am_i   = read_reg(REG_WHO_AM_I);
	g_icm42688.pwr_mgmt0  = read_reg(REG_PWR_MGMT0);
	g_icm42688.gyro_cfg0  = read_reg(REG_GYRO_CONFIG0);
	g_icm42688.accel_cfg0 = read_reg(REG_ACCEL_CONFIG0);
	g_icm42688.gyro_cfg1  = read_reg(REG_GYRO_CONFIG1);

	/* ── 零偏校准：静止采集（12B 布局: gx=6/7, gy=8/9, gz=10/11） ── */
	for (sample = 0U; sample < BIAS_SAMPLES; sample++)
	{
		read_regs(REG_ACCEL_DATA_X1, data, 12U);
		sum_x += (float)make_i16(data[6], data[7]) * GYRO_SCALE_DPS;
		sum_y += (float)make_i16(data[8], data[9]) * GYRO_SCALE_DPS;
		sum_z += (float)make_i16(data[10], data[11]) * GYRO_SCALE_DPS;
		Delay_ms(5U);
	}
	g_icm42688.gyro_bias_x = sum_x / (float)BIAS_SAMPLES;
	g_icm42688.gyro_bias_y = sum_y / (float)BIAS_SAMPLES;
	g_icm42688.gyro_bias_z = sum_z / (float)BIAS_SAMPLES;

	g_icm42688.yaw         = 0.0f;
	g_icm42688.initialized = 1U;
	return 1U;
}

/* ══════════════════════════════════════════════════════════════════════
 * ICM42688_ReadSensor — 读六轴 + 更新姿态
 *
 * 每 2ms 调用一次（500Hz）。
 *   Roll  = atan2(acc_y, acc_z)           ← 加速度重力方向
 *   Pitch = atan2(-acc_x, √(acc_y²+acc_z²)) ← 同上
 *   Yaw  += gyro_z × dt                   ← Z 轴角速度积分
 * ══════════════════════════════════════════════════════════════════════ */
void ICM42688_ReadSensor(void)
{
	uint8_t frame[12];
	uint8_t i;
	float   norm;

	if (g_icm42688.initialized == 0U) { return; }

	/*
	 * 布局(实测,与 OmniM0 一致): 0x1F 起 12 字节连续块
	 *   [0..5]=加速度  [6..7]=gx  [8..9]=gy  [10..11]=gz
	 * 实测证据: gz 在偏移 10/11(0x29/0x2A) 有数据, 而非0x2B/0x2C。
	 */
	read_regs(REG_ACCEL_DATA_X1, frame, sizeof(frame));

	/* 诊断: 保留 buf 供原始帧查看 */
	for (i = 0U; i < sizeof(frame); i++)
	{
		g_icm42688.dbg_frame[i] = frame[i];
	}
	g_icm42688.int_status = read_reg(0x30U);

	/* ── 记录原始 LSB（诊断用，注意：与 OmniM0 一致，不在驱动内翻转轴） ── */
	g_icm42688.raw_ax = make_i16(frame[0], frame[1]);
	g_icm42688.raw_ay = make_i16(frame[2], frame[3]);
	g_icm42688.raw_az = make_i16(frame[4], frame[5]);
	g_icm42688.raw_gx = make_i16(frame[6], frame[7]);
	g_icm42688.raw_gy = make_i16(frame[8], frame[9]);
	g_icm42688.raw_gz = make_i16(frame[10], frame[11]);

	/* ── 加速度: 原始 LSB → m/s² ── */
	g_icm42688.acc_x = (float)g_icm42688.raw_ax * ACCEL_SCALE_MS2;
	g_icm42688.acc_y = (float)g_icm42688.raw_ay * ACCEL_SCALE_MS2;
	g_icm42688.acc_z = (float)g_icm42688.raw_az * ACCEL_SCALE_MS2;

	/* ── 角速度: 原始 LSB → °/s，减零偏 ── */
	g_icm42688.gyro_x = (float)g_icm42688.raw_gx * GYRO_SCALE_DPS - g_icm42688.gyro_bias_x;
	g_icm42688.gyro_y = (float)g_icm42688.raw_gy * GYRO_SCALE_DPS - g_icm42688.gyro_bias_y;
	g_icm42688.gyro_z = (float)g_icm42688.raw_gz * GYRO_SCALE_DPS - g_icm42688.gyro_bias_z;

	/* ── 角速度死区 ── */
	if ((g_icm42688.gyro_x > -GYRO_DEADBAND_DPS) &&
	    (g_icm42688.gyro_x <  GYRO_DEADBAND_DPS)) { g_icm42688.gyro_x = 0.0f; }
	if ((g_icm42688.gyro_y > -GYRO_DEADBAND_DPS) &&
	    (g_icm42688.gyro_y <  GYRO_DEADBAND_DPS)) { g_icm42688.gyro_y = 0.0f; }
	if ((g_icm42688.gyro_z > -GYRO_DEADBAND_DPS) &&
	    (g_icm42688.gyro_z <  GYRO_DEADBAND_DPS)) { g_icm42688.gyro_z = 0.0f; }

	/* ── Roll / Pitch：加速度重力方向 ── */
	g_icm42688.roll  = atan2f(g_icm42688.acc_y, g_icm42688.acc_z) * DEG_PER_RAD;
	norm = sqrtf(g_icm42688.acc_y * g_icm42688.acc_y +
	             g_icm42688.acc_z * g_icm42688.acc_z);
	g_icm42688.pitch = atan2f(-g_icm42688.acc_x, norm) * DEG_PER_RAD;

	/* ── Yaw：Z 轴角速度积分 ── */
	g_icm42688.yaw += g_icm42688.gyro_z * SAMPLE_PERIOD_S;

	/* ── Yaw 规范到 ±180° ── */
	if      (g_icm42688.yaw >  180.0f) { g_icm42688.yaw -= 360.0f; }
	else if (g_icm42688.yaw < -180.0f) { g_icm42688.yaw += 360.0f; }
}

/* ══════════════════════════════════════════════════════════════════════
 * 数据获取（纯读缓存，不访问硬件）
 * ══════════════════════════════════════════════════════════════════════ */

void ICM42688_GetAttitude(float *roll, float *pitch, float *yaw)
{
	if (roll  != 0) { *roll  = g_icm42688.roll; }
	if (pitch != 0) { *pitch = g_icm42688.pitch; }
	if (yaw   != 0) { *yaw   = g_icm42688.yaw; }
}

void ICM42688_GetGyroscope(float *gx, float *gy, float *gz)
{
	if (gx != 0) { *gx = g_icm42688.gyro_x; }
	if (gy != 0) { *gy = g_icm42688.gyro_y; }
	if (gz != 0) { *gz = g_icm42688.gyro_z; }
}

void ICM42688_GetAccelerometer(float *ax, float *ay, float *az)
{
	if (ax != 0) { *ax = g_icm42688.acc_x; }
	if (ay != 0) { *ay = g_icm42688.acc_y; }
	if (az != 0) { *az = g_icm42688.acc_z; }}
