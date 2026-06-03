/**
 * @file    bsp_usart.h
 * @brief   USART1 DMA 发送 — 调试串口输出 (非阻塞)
 * @note    硬件: USART1, PA9(TX)/PA10(RX), 115200-8N1
 *          CubeMX 配置: USART1 使能 DMA TX + 全局中断
 *          缓冲区放在 RAM_D1 (.dma_buffer) 供 DMA 访问.
 *          连续调用时自动等待上一次 DMA 完成.
 */
#ifndef BSP_USART_H
#define BSP_USART_H

#include "struct_typedef.h"

/**
 * @brief  DMA 发送原始字节 (等待前次 DMA 完成)
 * @param  data  数据指针
 * @param  len   字节数 (≤255)
 */
void usart1_send(const uint8_t *data, uint16_t len);

/**
 * @brief  DMA 格式化打印 (类似 printf, 非阻塞)
 * @param  fmt  格式化字符串
 * @param  ...  可变参数
 * @note   内部缓冲 256 字节 (DMA 可访问), 超长截断.
 *         若前次 DMA 未完成会等待, 调试场景可接受.
 */
void usart1_print(const char *fmt, ...);

#endif
