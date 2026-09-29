#ifndef __F407_HW_SPI_H
#define __F407_HW_SPI_H

#include <stdint.h>

/*
 * f407_hw_spi.h — F407 硬件 SPI2 精简封装
 *
 * 提供 ICM42688 驱动所需的 init / transfer / cs 三个原语，
 * 函数签名与 ICM_SPI_*Fn 兼容，由 Enroll 层直接注入。
 */

#ifdef __cplusplus
extern "C" {
#endif

/* SPI2 外设初始化：SCK/MOSI/MISO 配 AF5 复用推挽，CS 配推挽输出。 */
void    F407_HW_SPI2_Init(void *sckPort, uint32_t sckPin,
                           void *mosiPort, uint32_t mosiPin,
                           void *misoPort, uint32_t misoPin,
                           void *csPort,   uint32_t csPin);

/* 全双工交换一字节：写入 tx，阻塞等待完成后返回收到的 rx。 */
uint8_t F407_HW_SPI2_Transfer(uint8_t tx);

/* 片选控制：0=拉低选通, 1=拉高释放。 */
void    F407_HW_SPI2_Cs(uint8_t level);

#ifdef __cplusplus
}
#endif

#endif /* __F407_HW_SPI_H */