/**
 * @file    bsp_usart.c
 * @brief   USART1 阻塞发送 — 调试串口输出
 * @note    硬件: USART1, PA9(TX)/PA10(RX), 115200-8N1
 *          使用 HAL_UART_Transmit 阻塞发送, 简单可靠.
 *          DMA 版本需额外配置 DMA 流中断, 当前调试阶段先用阻塞.
 */
#include "bsp_usart.h"
#include "main.h"
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>

extern UART_HandleTypeDef huart1;

/* ================================================================
 *  usart1_print — 阻塞格式化打印
 *
 *  函数功能:
 *    格式化字符串到栈缓冲区, 阻塞发送到 USART1.
 *    与 DMA 版本相比, 调试阶段更简单可靠, 无需额外中断配置.
 *
 *  函数参数:
 *    fmt: printf 风格格式化字符串
 *    ...: 可变参数
 *
 *  函数输出:
 *    串口 TX 引脚输出格式化字符串.
 *    函数返回时所有字节已发送完毕.
 *
 *  注意:
 *    缓冲区 256 字节, 超长截断.
 *    阻塞时间 ≈ len × 87μs @ 115200 (例如 100 字节 ≈ 8.7ms).
 *    调试打印已精简至每 5 秒 ~50 字节, 阻塞可接受.
 */
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


