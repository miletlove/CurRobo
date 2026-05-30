/**
 * @file    ws2812.c
 * @brief   WS2812B RGB LED 驱动实现 — SPI6 单线协议模拟
 * @note    硬件: DM-MC-board02 板载 WS2812B
 *          接口: SPI6 MOSI (PA7), SCK (PA5)
 *
 *          数据传输格式 (单颗灯珠):
 *          ┌────────┬────────┬────────┬──────────────┐
 *          │ 复位脉冲 │  G[7:0] │  R[7:0] │  B[7:0]     │
 *          │ (≥50μs) │  (MSB)  │  (MSB)  │  (MSB)      │
 *          └────────┴────────┴────────┴──────────────┘
 *          颜色顺序: GRB (绿→红→蓝), 与常见 RGB 库不同, 注意!
 *
 *          复位脉冲: 发送 100 字节 0x00, 持续 ≥128μs @ 8MHz (要求 ≥50μs)
 */
#include "ws2812.h"

/* ================================================================
 *  内部常量
 * ================================================================ */

/** 灯珠数量 (DM-MC-board02 板载 1 颗) */
#define WS2812_LED_COUNT    1

/** 复位脉冲长度 (字节数) — 要求 ≥50μs, 100 字节 @8MHz = 128μs */
#define WS2812_RESET_BYTES  100

/** 单颗灯珠数据长度 (3 色 × 8 bit/色 = 24 字节 SPI) */
#define WS2812_DATA_BYTES   24

/* ================================================================
 *  底层 SPI 发送
 * ================================================================ */

/**
 * @brief  通过 SPI 发送 WS2812 编码数据
 * @param  r  红色 (0~255)
 * @param  g  绿色 (0~255)
 * @param  b  蓝色 (0~255)
 * @note   数据格式: GRB, 每 bit 转换为 1 个 SPI 字节
 *         MSB 先发, 每个 SPI 字节右移 1 位后发送 (去掉首 bit 的冗余)
 *
 *         编码原理推导:
 *         设 SPI 时钟频率 f_SPI, 则单个 SPI bit 周期 T = 1/f_SPI.
 *
 *         WS2812 的 1 个数据 bit 需要 1.25μs (±600ns) 的波形:
 *         - 0 码: 高电平 T0H 后低电平 T0L
 *         - 1 码: 高电平 T1H 后低电平 T1L
 *
 *         用 SPI 的 8 个 bit 模拟 WS2812 的 1 个 bit:
 *         SPI 每字节 8 个 SCK 脉冲 → 8T 总时长 = 1μs @ 8MHz.
 *
 *         0 码 (WS2812_LOW_LEVEL=0xC0, >>1=0x60):
 *           SPI byte = 0b01100000
 *           MOSI 波形: ─┐┌──┐┌────────────
 *                        │2T│     6T      │  T0H=250ns, T0L=750ns
 *
 *         1 码 (WS2812_HIGH_LEVEL=0xF0, >>1=0x78):
 *           SPI byte = 0b01111000
 *           MOSI 波形: ─┐┌──────┐┌────────
 *                        │  4T  │   4T    │  T1H=500ns, T1L=500ns
 */
static void ws2812_send_single(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t txbuf[WS2812_DATA_BYTES];
    uint8_t dummy = 0;

    /*
     * 数据编码: GRB 顺序, 每颜色 8 bit, MSB 先发
     *
     *   txbuf 索引:  0  1  2  3  4  5  6  7 | 8  9 10 11 12 13 14 15 |16 17 18 19 20 21 22 23
     *   含义:         G7 G6 G5 G4 G3 G2 G1 G0|R7 R6 R5 R4 R3 R2 R1 R0|B7 B6 B5 B4 B3 B2 B1 B0
     *
     *   注意: 参考代码中 txbuf 的填充顺序是:
     *     txbuf[7-i]  = (G>>i)&1 ? HighLevel>>1 : LowLevel>>1  → i=0→7 对应 G7→G0
     *     txbuf[15-i] = (R>>i)&1 ? HighLevel>>1 : LowLevel>>1  → i=0→7 对应 R7→R0
     *     txbuf[23-i] = (B>>i)&1 ? HighLevel>>1 : LowLevel>>1  → i=0→7 对应 B7→B0
     *   所以 SPI 发送顺序: G7 G6 ... G0, R7 R6 ... R0, B7 B6 ... B0 (GRB, MSB first)
     */
    for (int i = 0; i < 8; i++)
    {
        txbuf[7 - i]  = (((g >> i) & 0x01) ? WS2812_HIGH_LEVEL : WS2812_LOW_LEVEL) >> 1;
        txbuf[15 - i] = (((r >> i) & 0x01) ? WS2812_HIGH_LEVEL : WS2812_LOW_LEVEL) >> 1;
        txbuf[23 - i] = (((b >> i) & 0x01) ? WS2812_HIGH_LEVEL : WS2812_LOW_LEVEL) >> 1;
    }

    /*
     * 发送流程:
     * 1. 发送 0 字节空帧 (清除 SPI 状态, 产生复位前间隙)
     * 2. 等待 SPI 空闲
     * 3. 发送 24 字节颜色编码数据
     * 4. 发送 100 字节 0x00 作为复位脉冲 (RES ≥ 50μs)
     */
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, &dummy, 0, 0xFFFF);
    while (WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY);

    HAL_SPI_Transmit(&WS2812_SPI_UNIT, txbuf, WS2812_DATA_BYTES, 0xFFFF);

    for (int i = 0; i < WS2812_RESET_BYTES; i++)
    {
        HAL_SPI_Transmit(&WS2812_SPI_UNIT, &dummy, 1, 0xFFFF);
    }
}

/* ================================================================
 *  API 实现 — 基础控制
 * ================================================================ */

/**
 * @brief  设置单个 WS2812 灯珠颜色 (GRB 格式, 立即刷新)
 * @param  r  红色分量 (0~255)
 * @param  g  绿色分量 (0~255)
 * @param  b  蓝色分量 (0~255)
 *
 *         函数调用示例:
 *         @code
 *         WS2812_Ctrl(255, 0, 0);   // 红色
 *         WS2812_Ctrl(0, 255, 0);   // 绿色
 *         WS2812_Ctrl(0, 0, 255);   // 蓝色
 *         @endcode
 *
 *         输出结果: WS2812 灯珠显示对应颜色, 阻塞约 130μs @ 8MHz SPI
 */
void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b)
{
    ws2812_send_single(r, g, b);
}

/**
 * @brief  通过颜色结构体设置 LED
 * @param  color  颜色结构体指针
 *
 *         使用预定义颜色宏:
 *         @code
 *         WS2812_SetColor(&WS2812_COLOR_RED);
 *         WS2812_SetColor(&WS2812_COLOR_CYAN);
 *         @endcode
 */
void WS2812_SetColor(const WS2812_Color_t *color)
{
    if (color == NULL) return;
    ws2812_send_single(color->r, color->g, color->b);
}

/**
 * @brief  关闭 LED
 */
void WS2812_Off(void)
{
    ws2812_send_single(0, 0, 0);
}

/* ================================================================
 *  API 实现 — 灯效
 * ================================================================ */

/**
 * @brief  简易呼吸灯效果 (非阻塞)
 * @param  period_ms  呼吸周期 (ms)
 * @retval 当前亮度值 (0~255)
 *
 *         亮度曲线: 三角波 (线性上升→线性下降)
 *
 *         数学推导:
 *         设 t = HAL_GetTick() % period_ms, T = period_ms
 *         归一化相位 φ = t / T, φ ∈ [0, 1)
 *
 *                         ┌ 2φ          , φ ∈ [0, 0.5)   (上升段)
 *         亮度系数 α(φ) = ┤
 *                         └ 2(1-φ)      , φ ∈ [0.5, 1)   (下降段)
 *
 *         brightness = α(φ) × 255
 *
 *         函数参数:
 *         - period_ms: 完整呼吸周期 (亮→灭→亮), 典型值 3000ms
 *           period_ms 越小呼吸越快, period_ms 越大呼吸越慢
 *
 *         函数输出:
 *         返回 0~255 的亮度值, 同时将 LED 设为纯蓝色呼吸效果。
 *         如需自定义颜色, 可自行组合调用。
 *
 *         使用示例:
 *         @code
 *         while (1) {
 *             WS2812_Breathing(3000);   // 3 秒一个呼吸周期
 *             HAL_Delay(20);            // 20ms 更新一次
 *         }
 *         @endcode
 */
uint8_t WS2812_Breathing(uint32_t period_ms)
{
    if (period_ms == 0) return 0;

    uint32_t now  = HAL_GetTick();
    uint32_t phase = now % period_ms;
    uint32_t half  = period_ms / 2;

    uint8_t brightness;

    if (phase < half)
    {
        /* 上升段: 0 → 255 */
        brightness = (uint8_t)((uint32_t)phase * 255 / half);
    }
    else
    {
        /* 下降段: 255 → 0 */
        brightness = (uint8_t)(((uint32_t)(period_ms - phase) * 255) / half);
    }

    /* 蓝色呼吸灯 — 可根据需要修改颜色 */
    ws2812_send_single(0, 0, brightness);
    return brightness;
}

/**
 * @brief  HSV 转 RGB (内部辅助函数)
 *
 *         转换公式推导 (基于 HSV 六边形模型):
 *
 *         输入: h ∈ [0, 360) 色相角
 *               s ∈ [0, 1]   饱和度
 *               v ∈ [0, 1]   明度
 *
 *         1. 计算色相扇区:
 *            sector = ⌊h / 60⌋,  sector ∈ {0,1,2,3,4,5}
 *            frac   = h/60 - sector,  frac ∈ [0, 1)
 *
 *         2. 计算基色分量:
 *            p = v × (1 - s)
 *            q = v × (1 - s × frac)
 *            t = v × (1 - s × (1 - frac))
 *
 *         3. 根据扇区映射到 RGB:
 *            sector 0: (v, t, p)   sector 3: (p, q, v)
 *            sector 1: (q, v, p)   sector 4: (t, p, v)
 *            sector 2: (p, v, t)   sector 5: (v, p, q)
 *
 *         最终输出: r,g,b ∈ [0, 255]
 */
static void hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v,
                       uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (s == 0)
    {
        /* 饱和度为 0 → 灰度 */
        *r = *g = *b = v;
        return;
    }

    uint8_t sector = h / 60;             /* 扇区 0~5              */
    uint8_t frac   = (uint8_t)(((uint32_t)(h % 60) * 255) / 60); /* 小数部分 [0,255]  */

    uint8_t p = (uint8_t)((uint16_t)v * (255 - s) / 255);             /* v × (1-s)       */
    uint8_t q = (uint8_t)((uint16_t)v * (255 - ((uint16_t)s * frac / 255)) / 255); /* v × (1-s×frac)   */
    uint8_t t = (uint8_t)((uint16_t)v * (255 - ((uint16_t)s * (255 - frac) / 255)) / 255);

    switch (sector)
    {
    case 0: *r = v; *g = t; *b = p; break;
    case 1: *r = q; *g = v; *b = p; break;
    case 2: *r = p; *g = v; *b = t; break;
    case 3: *r = p; *g = q; *b = v; break;
    case 4: *r = t; *g = p; *b = v; break;
    case 5: *r = v; *g = p; *b = q; break;
    default: *r = *g = *b = 0; break;
    }
}

/**
 * @brief  彩虹渐变效果 (非阻塞, 周期性调用)
 * @param  speed  变化速度 (1~255), 越大越快
 *
 *         原理: 色相 H 随时间线性循环变化 (H ∈ [0, 360)), 保持 S=V=255 (最大鲜艳度)
 *         H(t) = (H(t-Δt) + speed) % 360
 *
 *         函数参数:
 *         - speed: 色相每次步进的角度值
 *           speed=1 → 360 次调用走完一圈 (约 7.2s @ 20ms 间隔)
 *           speed=5 →  72 次调用走完一圈 (约 1.4s @ 20ms 间隔)
 *
 *         使用示例:
 *         @code
 *         while (1) {
 *             WS2812_Rainbow(3);     // 中速彩虹
 *             HAL_Delay(20);         // 20ms 更新一次
 *         }
 *         @endcode
 */
void WS2812_Rainbow(uint8_t speed)
{
    static uint16_t hue = 0;   /* 色相角度 0~359 */

    uint8_t r, g, b;
    hsv_to_rgb(hue, 255, 255, &r, &g, &b);
    ws2812_send_single(r, g, b);

    hue += speed;
    if (hue >= 360) hue -= 360;
}
