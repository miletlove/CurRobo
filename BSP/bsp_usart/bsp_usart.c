/**
 * @file    bsp_usart.c
 * @brief   USART1 DMA 发送 — 调试串口输出 (非阻塞)
 * @note    硬件: USART1, PA9(TX)/PA10(RX), 115200-8N1
 *          使用 HAL_UART_Transmit_DMA, 缓冲区放在 RAM_D1 (DMA 可访问).
 *          发送期间若再次调用 usart1_print, 会等待上一次 DMA 完成.
 */
#include "bsp_usart.h"
#include "main.h"
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

/* DMA 发送缓冲区 — 必须放在 DMA 可访问的 RAM_D1 (H7 的 DTCM 对 DMA 不可见) */
static uint8_t usart1_tx_buf[256] __attribute__((section(".dma_buffer"), aligned(32)));

/* DMA 发送完成标志 (HAL_UART_TxCpltCallback 中置位) */
static volatile uint8_t usart1_tx_done = 1;

/* ================================================================
 *  HAL_UART_TxCpltCallback — DMA 发送完成回调
 * ================================================================ */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        usart1_tx_done = 1;
    }
}

/* ================================================================
 *  usart1_send_dma_wait — 等待上一次 DMA 完成
 * ================================================================ */
static void usart1_dma_wait(void)
{
    uint32_t timeout = 10000;  /* ~10ms @ 240MHz, 实际 115200 波特率下 256B ≈ 22ms */
    while (!usart1_tx_done && timeout > 0)
    {
        timeout--;
    }
    /* 超时强制重置 (避免死锁) */
    if (timeout == 0)
    {
        HAL_UART_AbortTransmit(&huart1);
        usart1_tx_done = 1;
    }
}

/* ================================================================
 *  usart1_send — DMA 发送原始字节
 * ================================================================ */
void usart1_send(const uint8_t *data, uint16_t len)
{
    if (len == 0 || len > 255) return;

    usart1_dma_wait();
    memcpy(usart1_tx_buf, data, len);
    usart1_tx_done = 0;
    HAL_UART_Transmit_DMA(&huart1, usart1_tx_buf, len);
}

/* ================================================================
 *  usart1_print — DMA 格式化打印 (非阻塞)
 *
 *  函数功能:
 *    格式化字符串到 DMA 缓冲区, 启动 DMA 发送.
 *    若上一次 DMA 未完成, 等待其完成后再发送 (避免数据覆盖).
 *
 *  函数参数:
 *    fmt: printf 风格格式化字符串
 *    ...: 可变参数
 *
 *  函数输出:
 *    串口 TX 引脚输出格式化字符串.
 *    函数返回时 DMA 已启动 (非阻塞), 数据在后台发送.
 *
 *  注意:
 *    缓冲区 256 字节, 超长截断.
 *    在 DMA 完成前再次调用会等待 (阻塞), 调试场景可接受.
 */
void usart1_print(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf((char *)usart1_tx_buf, sizeof(usart1_tx_buf), fmt, ap);
    va_end(ap);

    if (len <= 0) return;
    if (len > 255) len = 255;

    usart1_dma_wait();
    usart1_tx_done = 0;
    HAL_UART_Transmit_DMA(&huart1, usart1_tx_buf, (uint16_t)len);
}


