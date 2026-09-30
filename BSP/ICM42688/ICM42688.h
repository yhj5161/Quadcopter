#ifndef __ICM42688_H
#define __ICM42688_H

#include <stdint.h>

/*
 * ICM42688.h — ICM-42688-P 6 轴 IMU 驱动
 *
 * 底层 SPI 通过函数指针注入(见 ICM42688_CtrlConfig_t)，驱动本身不感知
 * 走的是软件 SPI 还是硬件 SPI；当前板上注册为 F407 硬件 SPI2。
 * 采样率 500Hz(ICM ODR 1kHz)，加速度 ±16g、陀螺 ±2000dps。
 *
 * 姿态解算内置于 ICM42688_ReadSensor():
 *   Roll / Pitch 由加速度重力方向反算
 *   Yaw           由 Z 轴角速度积分(无磁力计修正,会漂移)
 * 输出为浮点 °/s / m/s² / 度,可直接用于飞控。
 */

/* WHO_AM_I 回读值 */
#define ICM42688_WHO_AM_I_VALUE  (0x47U)

/* SPI 操作回调：由 Enroll 层注入，ICM 驱动不感知底层是软/硬 SPI。 */
typedef void    (*ICM_SPI_InitFn)(void *sckPort, uint32_t sckPin,
                                   void *mosiPort, uint32_t mosiPin,
                                   void *misoPort, uint32_t misoPin,
                                   void *csPort,   uint32_t csPin);
typedef uint8_t (*ICM_SPI_TransferFn)(uint8_t tx);
typedef void    (*ICM_SPI_CsFn)(uint8_t level);

/* 引脚注册结构: SPI 操作通过函数指针注入，实现与底层解耦。 */
typedef struct
{
	void     *sckPort;  uint32_t sckPin;
	void     *mosiPort; uint32_t mosiPin;
	void     *misoPort; uint32_t misoPin;
	void     *csPort;   uint32_t csPin;

	ICM_SPI_InitFn     initFn;
	ICM_SPI_TransferFn transferFn;
	ICM_SPI_CsFn       csFn;
} ICM42688_CtrlConfig_t;

/*
 * 传感器数据结构（浮点输出，主循环 500Hz 调用）。
 * 加速度: m/s²，角速度: °/s（已减零偏），姿态角: 度。
 */
typedef struct
{
	float acc_x;     float acc_y;     float acc_z;
	float gyro_x;    float gyro_y;    float gyro_z;
	float roll;      float pitch;     float yaw;
	float gyro_bias_x, gyro_bias_y, gyro_bias_z;
	uint8_t initialized;

	/* 原始 LSB 诊断值：换算前的原始读数（每次 ReadSensor 刷新） */
	int16_t raw_ax, raw_ay, raw_az;
	int16_t raw_gx, raw_gy, raw_gz;

	/* 配置寄存器读回值（Init 后填充一次，用于确认配置写入是否生效） */
	uint8_t who_am_i;
	uint8_t pwr_mgmt0;
	uint8_t gyro_cfg0;
	uint8_t accel_cfg0;
	uint8_t gyro_cfg1;
} ICM42688_Data_t;

extern ICM42688_Data_t g_icm42688;

#ifdef __cplusplus
extern "C" {
#endif

/* 登记板上 ICM 引脚(由 Enroll_ICM42688_Register 调用)。 */
void ICM42688_Register(const ICM42688_CtrlConfig_t *configTable, uint8_t count);

/*
 * 初始化 + 零偏校准（传感器必须保持静止）。
 * 返回 1 成功, 0 失败(WHO_AM_I 不对 / 未注册)。
 */
uint8_t ICM42688_Init(void);

/*
 * 读六轴 + 更新姿态角：每 2ms 调用一次（500Hz）。
 * Roll/Pitch 由加速度反算, Yaw 由 Z 轴角速度积分。
 */
void ICM42688_ReadSensor(void);

/* 从缓存获取姿态角(度),不访问 SPI。传入 NULL 忽略对应轴。 */
void ICM42688_GetAttitude(float *roll, float *pitch, float *yaw);

/* 从缓存获取角速度(°/s, 已减零偏)。传入 NULL 忽略对应轴。 */
void ICM42688_GetGyroscope(float *gx, float *gy, float *gz);

/* 从缓存获取加速度(m/s²)。传入 NULL 忽略对应轴。 */
void ICM42688_GetAccelerometer(float *ax, float *ay, float *az);

#ifdef __cplusplus
}
#endif

#endif /* __ICM42688_H */