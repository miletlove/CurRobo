/**
 * @file    bsp_usart.h
 * @brief   USART1 DMA 发送 — 调试串口输出
 * @note    硬件: USART1, PA9(TX)/PA10(RX), 115200-8N1
 *          使用 DMA1_Stream3 + HAL_UART_Transmit_DMA 非阻塞发送
 *          依赖 HAL_UART_TxCpltCallback 管理 DMA 完成标志
 */
#ifndef BSP_USART_H
#define BSP_USART_H

#include "struct_typedef.h"

/**
 * @brief  DMA 非阻塞发送原始字节
 * @param  data  数据指针
 * @param  len   字节数 (≤256)
 * @note   内部自动等待上次 DMA 完成, 双缓冲保证数据安全
 */
void usart1_send(const uint8_t *data, uint16_t len);

/**
 * @brief  DMA 非阻塞格式化打印 (类似 printf)
 * @param  fmt  格式化字符串
 * @param  ...  可变参数
 * @note   内部缓冲 256 字节, 超长截断
 */
void usart1_print(const char *fmt, ...);

#endif
