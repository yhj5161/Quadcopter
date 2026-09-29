#include "Control_Task.h"

#include "tim.h"
#include "usart.h"
#include "My_Usart/My_Usart.h"
#include "KEY.h"
#include "LED.h"

/* 程序运行的时间戳（s） */
uint32_t Timer_Bsp_t = 0;

/* printf节拍 */
volatile uint8_t print_task_flag = 0;

/* 20ms 步态节拍：1ms 中断累加，满 20 由 ControlTask 消费 */
volatile uint8_t s_tick20ms = 0U;

/* UART4 帧解析 → OLED 显示。
 * 协议：s12,-34,56e —— 's' 包头、',' 分隔各数、'e' 包尾。
 * 非法帧整体丢弃；解析结果放 g_rxFrame[]，置 g_rxFrameReady=1，由 DisplayTask 消费后清 0。
 * 解析状态机复用 My_Usart 模块（usart_Dispose_Data）。 */
#define RX_FRAME_MAX  10U              /* 一帧最多 10 个数 */

/* static USART_DataType s_uart4Dec;      UART4 帧解析 —— 硬件未接暂时关闭 */
static USART_DataType s_uart1Dec;      /* USART1 帧解析状态实例 */

volatile int16_t g_rxFrame[RX_FRAME_MAX];
volatile uint8_t g_rxFrameCount;
volatile uint8_t g_rxFrameReady;

/*
 * API_TIM3: 1ms -> Key + printf + time
 */
void Control_Task_Housekeeping_Callback(API_TIM_Id_t id)
{
	static uint8_t printf_tick = 0U;
	static uint16_t time_t = 0U;

	if (id != API_TIM3)
	{
		return;
	}

	Key_Tick();

	printf_tick++;
	time_t++;
	s_tick20ms++;

	if (printf_tick >= 50U)
	{
		printf_tick = 0U;
		print_task_flag = 1U;
	}

	if (time_t >= 1000U)
	{
		time_t = 0U;
		Timer_Bsp_t++;
	}
}

/*
 * USART 中断回调：读取 RX 字节并分发到对应模块。
 */
void Control_Task_USART_Callback(API_USART_Id_t id)
{
	uint32_t data;
	uint8_t rxValid;

	data = 0U;
	rxValid = 0U;
	usart_irq_dispatch_by_id(id, &data, &rxValid);
	if (rxValid != 0U)
	{
		if (id == API_USART1)
		{
			/* USART1：测试遥控 LED1 —— s0,e=灭, s1,e=亮 */
			uint8_t c = (uint8_t)data;

			usart_Dispose_Data(USART1, &s_uart1Dec, c);
			if (s_uart1Dec.state == 2U)
			{
				int16_t cmd = USART_Deal(&s_uart1Dec, (int8_t)0);
				if (cmd == 0)
				{
					LED_Control(LED1, LED_LOW);
				}
				else if (cmd == 1)
				{
					LED_Control(LED1, LED_HIGH);
				}
				s_uart1Dec.state = 0U;
			}
		}
	}
}