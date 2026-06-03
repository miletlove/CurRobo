/**
 * @file    bsp_usart.c
 * @brief   USART1 DMA 发送 — 调试串口输出
 * @note    硬件: USART1, PA9(TX)/PA10(RX), 115200-8N1
 *          CubeMX 配置: DMA1_Stream3, USART1 全局中断已开启
 *          使用 HAL_UART_Transmit_DMA 非阻塞发送,
 *          内部用双缓冲 + 忙等待保证数据完整性.
 */
#include "bsp_usart.h"
#include "main.h"
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

/* ================================================================
 *  DMA 发送管理
 * ================================================================ */
#define USART1_DMA_BUF_SIZE  256
static char  g_tx_buf[2][USART1_DMA_BUF_SIZE]; /* 双缓冲 */
static uint8_t g_tx_buf_idx = 0;               /* 当前使用的缓冲索引 */
static volatile uint8_t g_tx_dma_busy = 0;     /* DMA 传输忙标志 */

/**
 * @brief  USART1 DMA 发送完成回调 (HAL 调用, ISR 上下文)
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
        g_tx_dma_busy = 0;
}

/**
 * @brief  等待 DMA 发送完成 (超时保护)
 */
static void usart1_dma_wait(void)
{
    uint32_t timeout = 10000;  /* ~10ms */
    while (g_tx_dma_busy && --timeout) { __NOP(); }
    g_tx_dma_busy = 0;  /* 超时强制清除 */
}

/* ================================================================
 *  API
 * ================================================================ */

void usart1_send(const uint8_t *data, uint16_t len)
{
    if (len == 0 || len > USART1_DMA_BUF_SIZE) return;

    /* 等待上一次 DMA 完成 */
    usart1_dma_wait();

    /* 拷贝到当前缓冲 */
    memcpy(g_tx_buf[g_tx_buf_idx], data, len);

    /* 启动 DMA 发送 */
    g_tx_dma_busy = 1;
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)g_tx_buf[g_tx_buf_idx], len);

    /* 切换到另一个缓冲 (下次不会覆盖正在发送的数据) */
    g_tx_buf_idx ^= 1;
}

void usart1_print(const char *fmt, ...)
{
    static char buf[USART1_DMA_BUF_SIZE];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (len > 0)
    {
        uint16_t slen = (uint16_t)(len < USART1_DMA_BUF_SIZE ? len
                                 : USART1_DMA_BUF_SIZE - 1);
        usart1_send((const uint8_t *)buf, slen);
    }
}

