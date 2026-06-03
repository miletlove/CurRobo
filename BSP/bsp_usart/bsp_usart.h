/**
 * @file    bsp_usart.h
 * @brief   USART1 阻塞发送 — 调试串口输出
 * @note    硬件: USART1, PA9(TX)/PA10(RX), 115200-8N1
 *          使用 HAL_UART_Transmit 阻塞发送, 简单可靠.
 */
#ifndef BSP_USART_H
#define BSP_USART_H

#include "struct_typedef.h"

/**
 * @brief  阻塞格式化打印 (类似 printf)
 * @param  fmt  格式化字符串
 * @param  ...  可变参数
 * @note   内部缓冲 256 字节, 超长截断.
 *         阻塞时间 ≈ len × 87μs @ 115200.
 */
void usart1_print(const char *fmt, ...);

#endif
