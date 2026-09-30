#ifndef __407_HW_CONFIG_H
#define __407_HW_CONFIG_H

#include "f407_gpio.h" /* GPIO_Pin_x + GPIOA/B 原语，其余枚举值在 Enroll.c 展开时由 Enroll_Internal.h 提供 */

/*
 * 407_hw_config.h 板级硬件映射宏
 */

/* LED 板级映射：LED1=E2绿灯，LED2=E3红灯，LED3=PE4蓝灯 ,Buzzer1=B0 */
#define HW_LED_MAP(X) \
	X(LED1, GPIOE, GPIO_Pin_2) \
	X(LED2, GPIOE, GPIO_Pin_3) \
	X(LED3, GPIOE, GPIO_Pin_4) \
	X(Buzzer1, GPIOB, GPIO_Pin_0)

/* USART1 引脚定义：TX=PA9，RX=PA10（遥测打印 + 串口控制，AF7 复用） */
#define HW_USART1_TX_PORT GPIOA
#define HW_USART1_TX_PIN  GPIO_Pin_9
#define HW_USART1_RX_PORT GPIOA
#define HW_USART1_RX_PIN  GPIO_Pin_10

/* USART2 引脚定义：TX=PD5，RX=PD6 */
#define HW_USART2_TX_PORT GPIOD
#define HW_USART2_TX_PIN  GPIO_Pin_5
#define HW_USART2_RX_PORT GPIOD
#define HW_USART2_RX_PIN  GPIO_Pin_6

/* 无线串口 USART3 引脚定义：TX=PD8，RX=PD9 */
#define HW_USART3_TX_PORT GPIOD
#define HW_USART3_TX_PIN  GPIO_Pin_8
#define HW_USART3_RX_PORT GPIOD
#define HW_USART3_RX_PIN  GPIO_Pin_9

/* UART4 引脚定义：TX=PA0，RX=PA1（F407 UART4 备用映射 AF8，已注册未初始化） */
#define HW_UART4_TX_PORT GPIOA
#define HW_UART4_TX_PIN  GPIO_Pin_0
#define HW_UART4_RX_PORT GPIOA
#define HW_UART4_RX_PIN  GPIO_Pin_1

/* USART 板级映射 */
#define HW_USART_MAP(X) \
	X(API_USART1, API_USART_CORE_USART1, HW_USART1_TX_PORT, HW_USART1_TX_PIN, HW_USART1_RX_PORT, HW_USART1_RX_PIN) \
	X(API_USART2, API_USART_CORE_USART2, HW_USART2_TX_PORT, HW_USART2_TX_PIN, HW_USART2_RX_PORT, HW_USART2_RX_PIN) \
	X(API_USART3, API_USART_CORE_USART3, HW_USART3_TX_PORT, HW_USART3_TX_PIN, HW_USART3_RX_PORT, HW_USART3_RX_PIN) \
	X(API_USART4, API_USART_CORE_UART4,  HW_UART4_TX_PORT, HW_UART4_TX_PIN, HW_UART4_RX_PORT, HW_UART4_RX_PIN)

/* TIM 板级映射：逻辑槽位 API_TIM3 → 硬件 TIM5（1ms 节拍）。槽位名与硬件编号无关，只看本表。 */
#define HW_TIM_MAP(X) \
	X(API_TIM3, API_TIM_CORE_TIM5)

/* PWM 板级映射（无刷电机）：PE9 -> TIM1_CH1，PE11 -> TIM1_CH2，PE13 -> TIM1_CH3，PE14 -> TIM1_CH4 */
#define HW_PWM_MAP(X) \
	X(API_PWM_TIM1, API_PWM_CH1, API_PWM_CORE_TIM1, API_PWM_CORE_CH1, GPIOE, GPIO_Pin_9) \
	X(API_PWM_TIM1, API_PWM_CH2, API_PWM_CORE_TIM1, API_PWM_CORE_CH2, GPIOE, GPIO_Pin_11) \
	X(API_PWM_TIM1, API_PWM_CH3, API_PWM_CORE_TIM1, API_PWM_CORE_CH3, GPIOE, GPIO_Pin_13) \
	X(API_PWM_TIM1, API_PWM_CH4, API_PWM_CORE_TIM1, API_PWM_CORE_CH4, GPIOE, GPIO_Pin_14)

/* ADC 板级映射：PB1（ADC1_IN9），预留电池电压检测（尚未调用 API_ADC_GetValue） */
#define HW_ADC_MAP(X) \
	X(API_ADC1, API_ADC_CH9, GPIOB, GPIO_Pin_1)

/* 软件 I2C1 引脚定义：SCL=PB8，SDA=PB9（QMC5883P 磁力计 / BMP280 气压计）*/
#define HW_I2C1_SCL_PORT GPIOB
#define HW_I2C1_SCL_PIN  GPIO_Pin_8
#define HW_I2C1_SDA_PORT GPIOB
#define HW_I2C1_SDA_PIN  GPIO_Pin_9

/* I2C 板级映射：I2C1=传感器（I2C2 已移除，PA5/PA7 让给 NRF24L01） */
#define HW_I2C_MAP(X) \
	X(API_I2C1, HW_I2C1_SCL_PORT, HW_I2C1_SCL_PIN, HW_I2C1_SDA_PORT, HW_I2C1_SDA_PIN, 0, 0)

/* 软件 SPI 引脚定义：给 NRF24L01（bit-bang）。PA5/6/7 是硬件 SPI1 脚位（AF5），将来可升级为硬件 SPI。
   "SPI2" 只是 API 槽位编号，与硬件 SPI2（PB13-15，ICM 在用）无关。 */
#define HW_SPI2_SCK_PORT  GPIOA
#define HW_SPI2_SCK_PIN   GPIO_Pin_5

#define HW_SPI2_MOSI_PORT GPIOA
#define HW_SPI2_MOSI_PIN  GPIO_Pin_7

#define HW_SPI2_MISO_PORT GPIOA
#define HW_SPI2_MISO_PIN  GPIO_Pin_6

#define HW_SPI2_CS_PORT   GPIOC
#define HW_SPI2_CS_PIN    GPIO_Pin_4

/* SPI 板级映射：逻辑槽位 API_SPI2（编号）→ NRF24L01，底层固定走 soft_spi_hal */
#define HW_SPI_MAP(X) \
	X(API_SPI2, HW_SPI2_CS_PORT, HW_SPI2_CS_PIN, \
	  HW_SPI2_SCK_PORT, HW_SPI2_SCK_PIN, \
	  HW_SPI2_MOSI_PORT, HW_SPI2_MOSI_PIN, \
	  HW_SPI2_MISO_PORT, HW_SPI2_MISO_PIN, 0, 0, 0, 0)

/* NRF24L01 控制引脚定义：CE=PC5 */
#define HW_NRF24L01_CE_PORT GPIOC
#define HW_NRF24L01_CE_PIN  GPIO_Pin_5

/* NRF24L01 控制引脚映射：注册 1 组 CE */
#define HW_NRF24L01_CTRL_MAP(X) \
	X(HW_NRF24L01_CE_PORT, HW_NRF24L01_CE_PIN)

/* ICM42688P 引脚定义：CS=PB12, SCK=PB13, MISO=PB14, MOSI=PB15 (F407 硬件 SPI2, AF5) */
#define HW_ICM_SCK_PORT             GPIOB
#define HW_ICM_SCK_PIN              GPIO_Pin_13
#define HW_ICM_MOSI_PORT            GPIOB
#define HW_ICM_MOSI_PIN             GPIO_Pin_15
#define HW_ICM_MISO_PORT            GPIOB
#define HW_ICM_MISO_PIN             GPIO_Pin_14
#define HW_ICM_CS_PORT              GPIOB
#define HW_ICM_CS_PIN               GPIO_Pin_12

/* ICM42688 引脚映射：注册 1 组(纯引脚,无 EXTI——500Hz 轮询读取)。
   SPI 操作通过函数指针注入，由 Enroll 层绑定到 F407_HW_SPI2_Init/Transfer/Cs。 */
#define HW_ICM42688_MAP(X) \
	X(HW_ICM_SCK_PORT, HW_ICM_SCK_PIN, \
	  HW_ICM_MOSI_PORT, HW_ICM_MOSI_PIN, \
	  HW_ICM_MISO_PORT, HW_ICM_MISO_PIN, \
	  HW_ICM_CS_PORT, HW_ICM_CS_PIN)
#define HW_ICM42688_COUNT           1U

/* 当前板子上注册了 3 个 LED + 1 个蜂鸣器 */
#define HW_LED_COUNT  4U
/* 当前板子上注册了 4 路 USART */
#define HW_USART_COUNT  4U
/* 当前板子上注册了 1 路 TIM */
#define HW_TIM_COUNT  1U
/* 当前板子上注册了 4 路 PWM 通道 */
#define HW_PWM_COUNT  4U
/* 当前板子上注册了 1 路 ADC 通道 */
#define HW_ADC_COUNT  1U
/* 当前板子上注册了 1 路软件 I2C */
#define HW_I2C_COUNT  1U
/* 当前板子上注册了 1 路软件 SPI */
#define HW_SPI_COUNT  1U
/* 当前板子上注册了 1 组 NRF24L01 控制引脚 */
#define HW_NRF24L01_CTRL_COUNT  1U

#endif /* __407_HW_CONFIG_H */
