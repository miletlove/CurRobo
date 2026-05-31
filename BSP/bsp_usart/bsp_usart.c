/**
 * @file    bsp_usart.c
 * @brief   USART1 阻塞发送 — 调试串口输出
 * @note    硬件: USART1, PA9(TX)/PA10(RX), 115200-8N1
 *          使用 HAL_UART_Transmit 阻塞发送, 适用于调试输出.
 *          不依赖 DMA, 简单可靠.
 */
#include "bsp_usart.h"
#include "main.h"
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>

extern UART_HandleTypeDef huart1;

void usart1_send(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 100);
}

void usart1_print(const char *fmt, ...)
{
    static char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (len > 0)
        HAL_UART_Transmit(&huart1, (uint8_t *)buf,
                          (uint16_t)(len < 256 ? len : 255), 100);
}


