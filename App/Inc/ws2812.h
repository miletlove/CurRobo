/**
 * @file    ws2812.h
 * @brief   WS2812B RGB LED 驱动接口 — 基于 SPI6 单线协议
 * @note    适用于达妙科技 DM-MC-board02 板载 RGB LED
 *          使用 SPI6 MOSI (PA7) 模拟 WS2812 单线时序
 *          GRB 数据顺序, MSB 先发
 */
#ifndef __WS2812_H__
#define __WS2812_H__

#include "main.h"
#include "spi.h"
#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  SPI 句柄宏 — 与 CubeMX 生成的 hspi6 绑定
 * ================================================================ */
#define WS2812_SPI_UNIT     hspi6

/* ================================================================
 *  WS2812 位编码定义
 *
 *  原理: 每 1 个 WS2812 数据 bit 用 SPI 的 1 个字节 (8 个 SCK 脉冲) 模拟
 *
 *  假定时钟频率 f_SPI = 8 MHz → T_bit = 125 ns:
 *
 *  0 码: 0xC0 >> 1 = 0x60 → MOSI 序列: 0 1 1 0 0 0 0 0
 *        ┌──┐┌──────────────┐
 *        │2T │     6T       │  T0H=250ns, T0L=750ns
 *        └──┘└──────────────┘
 *
 *  1 码: 0xF0 >> 1 = 0x78 → MOSI 序列: 0 1 1 1 1 0 0 0
 *        ┌──────┐┌──────────┐
 *        │  4T  │    4T     │  T1H=500ns, T1L=500ns
 *        └──────┘└──────────┘
 *
 *  WS2812B 时序要求 (参考):
 *    T0H: 300~450ns, T0L: ≥750ns
 *    T1H: 600~950ns, T1L: ≥200ns
 *    RES: ≥50μs
 *
 *  上述 8MHz 时序在实际 WS2812B 上容差范围内均可用。
 *  如需微调, 修改以下宏即可改变占空比。
 * ================================================================ */
#define WS2812_LOW_LEVEL    0xC0    /* 0 码: SPI 字节原值, 发送前 >>1 */
#define WS2812_HIGH_LEVEL   0xF0    /* 1 码: SPI 字节原值, 发送前 >>1 */

/* ================================================================
 *  灯效配置结构体
 * ================================================================ */

/** RGB 颜色值 (0~255 各通道) */
typedef struct {
    uint8_t r;   /* 红色通道 */
    uint8_t g;   /* 绿色通道 */
    uint8_t b;   /* 蓝色通道 */
} WS2812_Color_t;

/** 预定义常用颜色 */
#define WS2812_COLOR_RED       ((WS2812_Color_t){255,   0,   0})
#define WS2812_COLOR_GREEN     ((WS2812_Color_t){  0, 255,   0})
#define WS2812_COLOR_BLUE      ((WS2812_Color_t){  0,   0, 255})
#define WS2812_COLOR_YELLOW    ((WS2812_Color_t){255, 255,   0})
#define WS2812_COLOR_CYAN      ((WS2812_Color_t){  0, 255, 255})
#define WS2812_COLOR_MAGENTA   ((WS2812_Color_t){255,   0, 255})
#define WS2812_COLOR_WHITE     ((WS2812_Color_t){255, 255, 255})
#define WS2812_COLOR_ORANGE    ((WS2812_Color_t){255, 165,   0})
#define WS2812_COLOR_OFF       ((WS2812_Color_t){  0,   0,   0})

/* ================================================================
 *  API 声明
 * ================================================================ */

/**
 * @brief  设置单个 WS2812 灯珠颜色 (单颗, 立即刷新)
 * @param  r  红色分量 (0~255)
 * @param  g  绿色分量 (0~255)
 * @param  b  蓝色分量 (0~255)
 * @note   这是原始接口, 兼容参考例程的直接调用方式。
 *         内部使用阻塞式 SPI 发送, 约耗时 130μs @ 8MHz。
 *         如需多颗灯珠或非阻塞控制, 使用 ctrl_by_color 系列函数。
 */
void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief  通过颜色结构体设置灯珠颜色
 * @param  color  RGB 颜色结构体
 */
void WS2812_SetColor(const WS2812_Color_t *color);

/**
 * @brief  关闭 LED (全灭)
 */
void WS2812_Off(void);

/**
 * @brief  简易呼吸灯效果 (非阻塞, 需周期性调用)
 * @param  period_ms  呼吸周期 (ms), 典型值 2000~5000
 * @retval 当前亮度 (0~255), 可用于组合其他效果
 * @note   基于 HAL_GetTick() 实现, 在 while(1) 中每 10~50ms 调用一次
 */
uint8_t WS2812_Breathing(uint32_t period_ms);

/**
 * @brief  彩虹渐变效果 (非阻塞, 需周期性调用)
 * @param  speed  变化速度 (1~255), 越大越快, 典型值 1~5
 * @note   基于 HSV→RGB 转换, 在 while(1) 中每 10~50ms 调用一次
 */
void WS2812_Rainbow(uint8_t speed);

#ifdef __cplusplus
}
#endif

#endif /* __WS2812_H__ */
