/* Enroll 注册层，负责把板级资源注册到 BSP */
#include "Enroll.h"

/*系统sys层*/
#include "sys.h"
#include "Delay.h"

/*API层 MCU片内外设*/
#include "usart.h"
#include "tim.h"
#include "pwm.h"
#include "adc.h"

/*app应用层*/
#include "My_Usart/My_Usart.h"
#include "API_I2C.h"
#include "API_SPI.h"
#include "ICM42688.h"
#include "QMC5883P.h"
#include "Control_Task/Control_Task.h"

/*BSP硬件抽象层*/
#include "LED.h"
#include "KEY.h"
#include "NRF24L01.h"

/*FreeRTOS 内核*/
#include "FreeRTOS.h"
#include "task.h"

/*
 * ======================== FreeRTOS 任务规划 ========================
 *
 * 四轴飞行器框架，任务按周期轮询。
 *
 * ┌─────────────┬──────┬──────────────────────────────────────────┐
 * │ 任务        │ 优先级│ 职责                                     │
 * ├─────────────┼──────┼──────────────────────────────────────────┤
 * │ ControlTask │ +3   │ 飞控主循环 + 按键/NRF（1ms）              │
 * │ SensorTask  │ +2   │ 传感器读取（2ms）                         │
 * │ DisplayTask │ +1   │ 遥测串口输出（50ms）                      │
 * └─────────────┴──────┴──────────────────────────────────────────┘
 *
 * 任务栈大小单位是 word（1 word = 4 字节），512 word = 2KB。
 */
#define TASK_PRIO_CONTROL     (tskIDLE_PRIORITY + 3U)
#define TASK_PRIO_SENSOR      (tskIDLE_PRIORITY + 2U)
#define TASK_PRIO_DISPLAY     (tskIDLE_PRIORITY + 1U)

#define TASK_STACK_CONTROL    (512U)
#define TASK_STACK_SENSOR     (384U)
#define TASK_STACK_DISPLAY    (512U)

#define TASK_PERIOD_CONTROL   (1U)
#define TASK_PERIOD_SENSOR    (2U)
#define TASK_PERIOD_DISPLAY   (50U)

/*
 * 磁力计读取分频：QMC ODR 200Hz，但融合只需要低频航向，
 * 且软件 I2C 是阻塞式的（一次读约百微秒级），不适合塞进 2ms 节拍。
 * 25 × 2ms = 50ms → 20Hz，数据仍然新鲜，且不挤占 SensorTask。
 */
#define MAG_READ_DIV          (25U)

static void ControlTask(void *argument);
static void SensorTask(void *argument);
static void DisplayTask(void *argument);

/*
 * NRF24L01 接收状态：遥控器每帧发 4 字节（d0=RV速度, d1=RH转向, d2=按键码, d3=LV倍率）。
 */
static volatile uint8_t  s_nrfRxBuf[NRF24L01_RX_PACKET_WIDTH] = {0U, 0U, 0U, 0U};
static volatile uint32_t s_nrfRxCount;
static uint8_t s_nrfBlink;

int main(void)
{
	/* 系统时钟配置初始化 */
	SYS_Init();
	/* 注册层：注册相关资源，登记资源映射 */
	Enroll_USART_Register();
	Enroll_PWM_Register();
	Enroll_ADC_Register();
	Enroll_TIM_Register();
	Enroll_I2C_Register();
	Enroll_SPI_Register();
	Enroll_LED_Register();
	Enroll_KEY_Register();
	Enroll_NRF24L01_Register();
	Enroll_ICM42688_Register();

	/* 注册后绑定中断回调*/
	Enroll_USART_RegisterIrqHandler(Control_Task_USART_Callback);
	API_TIM_RegisterIrqHandler(API_TIM3, Control_Task_Housekeeping_Callback);

	/* 初始化层：初始化相关外设，启动硬件功能 */
	API_USART_Init(API_USART1, 115200U);	/* USART1: 串口打印 + 串口控制 */
	/* USART3 已关闭 */
	/* UART4 硬件未接，暂时关闭 */
	/* PWM 初始化：TIM1 四通道，50Hz（四轴电机常用 50~400Hz） */
	API_PWM_Init(API_PWM_TIM1, 400U - 1U, 8U - 1U);
	API_ADC_Init(API_ADC1);
	API_TIM_Init(API_TIM3, 1U); /* TIM3: 杂务节拍，每 1ms */

	/* 通信协议初始化 */
	API_I2C_Init();
	API_SPI_Init();
	App_I2C_ScanOnce();
	NRF24L01_Init();
	ICM42688_Init();

	/* QMC5883P 磁力计初始化 + 读 ID 验证 */
	QMC_Init();
	usart_printf(USART1, "QMC ID=0x%02X\r\n", (unsigned)QMC_GetID());

	/* BSP硬件抽象层初始化 */
	LED_Init(LED_LOW);
	KEY_Init();
	/* ======================== 创建任务，启动调度器 ======================== */
	(void)xTaskCreate(ControlTask, "control", TASK_STACK_CONTROL, NULL, TASK_PRIO_CONTROL, NULL);
	(void)xTaskCreate(SensorTask, "sensor", TASK_STACK_SENSOR, NULL, TASK_PRIO_SENSOR, NULL);
	(void)xTaskCreate(DisplayTask, "display", TASK_STACK_DISPLAY, NULL, TASK_PRIO_DISPLAY, NULL);

	vTaskStartScheduler();

	for (;;)
	{
	}
}

/*
 * 控制任务：1ms 轮询 —— 飞控主循环 + 按键 + NRF24L01 接收。
 */
static void ControlTask(void *argument)
{
	(void)argument;

	for (;;)
	{
		/* 按键：硬件自检，点亮/熄灭 LED（后续由飞控主循环取代） */
		key_Get();
		if (Key == 1U)
		{
			LED_Control(LED1, LED_HIGH);
			Key = 0U;
		}
		if (Key == 2U)
		{
			LED_Control(LED2, LED_HIGH);
		}
		if (Key == 3U)
		{
			LED_Control(LED3, LED_HIGH);
		}
		if (Key == 4U)
		{
			LED_Control(LED1, LED_LOW);
			LED_Control(LED2, LED_LOW);
			LED_Control(LED3, LED_LOW);
		}

		/* NRF24L01 接收 */
		if (NRF24L01_Receive() == 1U)
		{
			uint8_t i;
			for (i = 0U; i < NRF24L01_RX_PACKET_WIDTH; i++)
			{
				s_nrfRxBuf[i] = NRF24L01_RxPacket[i];
			}
			s_nrfRxCount++;
			s_nrfBlink ^= 1U;
			LED_Control(LED2, s_nrfBlink ? LED_HIGH : LED_LOW);
		}

		/* TODO: 飞控姿态控制（IMU 数据 → PID → PWM 输出） */

		vTaskDelay(pdMS_TO_TICKS(TASK_PERIOD_CONTROL));
	}
}

/*
 * 传感器任务：读取 IMU 并更新姿态角（每 2ms / 500Hz）。
 * 磁力计按 MAG_READ_DIV 分频读取（20Hz），读一次刷新缓存。
 * TODO: 磁力计轴对齐、零点标定、倾斜补偿、Yaw 融合；BMP280 气压计（定高）。
 *       注意：当前磁力计航向是"相对角度"，零点未标定，不可当绝对航向用。
 */
static void SensorTask(void *argument)
{
	(void)argument;
	TickType_t lastWake = xTaskGetTickCount();
	uint8_t    magDiv = 0U;

	for (;;)
	{
		ICM42688_ReadSensor();

		/* 磁力计 20Hz：分频读取，读一次即刷新缓存（显示/融合读缓存即可） */
		if (++magDiv >= MAG_READ_DIV)
		{
			magDiv = 0U;
			QMC_ReadSensor();
		}

		vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TASK_PERIOD_SENSOR));
	}
}

/*
 * 显示任务：遥测串口输出，最低优先级。
 */
static void DisplayTask(void *argument)
{
	(void)argument;

	for (;;)
	{
		usart_printf(USART1, "t=%lu\r\n", (unsigned long)Timer_Bsp_t);

		if (s_nrfRxCount != 0U)
		{
			usart_printf(USART1, "NRF cnt=%lu d0=%u d1=%u d2=%u d3=%u\r\n",
			             (unsigned long)s_nrfRxCount,
			             (unsigned int)s_nrfRxBuf[0], (unsigned int)s_nrfRxBuf[1],
			             (unsigned int)s_nrfRxBuf[2], (unsigned int)s_nrfRxBuf[3]);
		}

		usart_printf(USART1, "ICM R=%.1f P=%.1f Y=%.1f\r\n",
		             (double)g_icm42688.roll, (double)g_icm42688.pitch, (double)g_icm42688.yaw);

		/* 磁力计：只读缓存，不在打印任务里访问 I2C（读取在 SensorTask，20Hz） */
		int16_t qx, qy, qz;
		QMC_GetMag(&qx, &qy, &qz);
		usart_printf(USART1, "QMC H=%.1f\r\n", (double)QMC_GetAngle());
		usart_printf(USART1, "MAG x=%d y=%d z=%d\r\n", (int)qx, (int)qy, (int)qz);

		vTaskDelay(pdMS_TO_TICKS(TASK_PERIOD_DISPLAY));
	}
}

/* ======================== FreeRTOS 钩子函数 ======================== */

void vApplicationMallocFailedHook(void)
{
	taskDISABLE_INTERRUPTS();
	LED_Control(LED1, LED_HIGH);
	for (;;)
	{
	}
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	(void)xTask;
	(void)pcTaskName;
	taskDISABLE_INTERRUPTS();
	LED_Control(LED2, LED_HIGH);
	for (;;)
	{
	}
}
