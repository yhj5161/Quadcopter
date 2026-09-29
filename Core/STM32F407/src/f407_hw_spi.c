#include "f407_hw_spi.h"
#include "f407_gpio.h"

/* SPI2 寄存器映射(F407: base 0x40003800, APB1) */
typedef struct
{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
} F407_SPI_Regs_t;

#define F407_SPI2_BASE     (0x40003800UL)
#define F407_SPI2          ((F407_SPI_Regs_t *)F407_SPI2_BASE)

/* RCC 位 */
#define RCC_AHB1_GPIOB     (1UL << 1)
#define RCC_APB1_SPI2      (1UL << 14)

/* SPI 控制位 */
#define SPI_CR1_SPE        (1UL << 6)
#define SPI_CR1_BR_DIV8    (2UL << 3)
#define SPI_CR1_MSTR       (1UL << 2)
#define SPI_SR_RXNE        (1UL << 0)
#define SPI_SR_TXE         (1UL << 1)

/* 片选 GPIO(推挽输出),Init 后缓存以便 Cs() 操作。 */
static void     *s_csPort;
static uint32_t  s_csPin;

/* ───────────────────────────  SPI2 外设 + 引脚初始化 ─────────────────── */
void F407_HW_SPI2_Init(void *sckPort, uint32_t sckPin,
                        void *mosiPort, uint32_t mosiPin,
                        void *misoPort, uint32_t misoPin,
                        void *csPort,   uint32_t csPin)
{
	F407_GPIO_Regs_t *gpio;
	uint32_t pinIndex;
	uint32_t shift;

	/* 1) 开启 GPIOB + SPI2 时钟 */
	F407_RCC->AHB1ENR |= RCC_AHB1_GPIOB;
	F407_RCC->APB1ENR |= RCC_APB1_SPI2;

	/* 2) GPIO 复用配置: SCK(PB13) MOSI(PB15) MISO(PB14) → AF5 */
	gpio = (F407_GPIO_Regs_t *)sckPort;   /* 三根是同一个 port(GPIOB) */
	(void)mosiPort;
	(void)misoPort;

	/* SCK PB13: AF5 推挽 高速 */
	pinIndex = F407_GPIO_PinIndex(sckPin);
	shift    = pinIndex * 2U;
	gpio->MODER   &= ~(0x3UL << shift);
	gpio->MODER   |=  (0x2UL << shift);    /* AF */
	gpio->OTYPER  &= ~(1UL << pinIndex);   /* PP */
	gpio->OSPEEDR &= ~(0x3UL << shift);
	gpio->OSPEEDR |=  (0x2UL << shift);    /* 高速 */
	gpio->AFRH    &= ~(0xFUL << ((pinIndex - 8U) * 4U));
	gpio->AFRH    |=  (5UL << ((pinIndex - 8U) * 4U));  /* AF5 */

	/* MISO PB14: AF5 输入(复用模式即输入) */
	pinIndex = F407_GPIO_PinIndex(misoPin);
	shift    = pinIndex * 2U;
	gpio->MODER   &= ~(0x3UL << shift);
	gpio->MODER   |=  (0x2UL << shift);
	gpio->OTYPER  &= ~(1UL << pinIndex);
	gpio->OSPEEDR &= ~(0x3UL << shift);
	gpio->OSPEEDR |=  (0x2UL << shift);
	gpio->AFRH    &= ~(0xFUL << ((pinIndex - 8U) * 4U));
	gpio->AFRH    |=  (5UL << ((pinIndex - 8U) * 4U));

	/* MOSI PB15: AF5 推挽 高速 */
	pinIndex = F407_GPIO_PinIndex(mosiPin);
	shift    = pinIndex * 2U;
	gpio->MODER   &= ~(0x3UL << shift);
	gpio->MODER   |=  (0x2UL << shift);
	gpio->OTYPER  &= ~(1UL << pinIndex);
	gpio->OSPEEDR &= ~(0x3UL << shift);
	gpio->OSPEEDR |=  (0x2UL << shift);
	gpio->AFRH    &= ~(0xFUL << ((pinIndex - 8U) * 4U));
	gpio->AFRH    |=  (5UL << ((pinIndex - 8U) * 4U));

	/* CS: 推挽输出 */
	F407_GPIO_InitOutput(csPort, csPin);

	/* 3) SPI2 寄存器 — master, mode0, 8-bit, MSB, BR÷8→5.25MHz, SSM=1 软件管理NSS */
	F407_SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_DIV8
	               | (1UL << 9)    /* SSM: 软件从机管理, 不依赖物理 NSS 引脚 */
	               | (1UL << 8);   /* SSI: 内部从机选中, 强制主机模式      */
	F407_SPI2->CR1 |= SPI_CR1_SPE;

	/* 缓存 CS 引脚供 Cs() 使用 */
	s_csPort = csPort;
	s_csPin  = csPin;
	F407_HW_SPI2_Cs(1U);   /* 默认释放 CS */
}

/* ─────────────────────────── 全双工交换 ─────────────────── */
uint8_t F407_HW_SPI2_Transfer(uint8_t tx)
{
	/* 等待 TXE */
	while ((F407_SPI2->SR & SPI_SR_TXE) == 0U) { }
	F407_SPI2->DR = tx;

	/* 等待 RXNE */
	while ((F407_SPI2->SR & SPI_SR_RXNE) == 0U) { }
	return (uint8_t)F407_SPI2->DR;
}

/* ─────────────────────────── 片选控制 ─────────────────── */
void F407_HW_SPI2_Cs(uint8_t level)
{
	F407_GPIO_Write(s_csPort, s_csPin, (level != 0U) ? 1U : 0U);
}