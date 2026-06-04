/**
 * @file    watchdog.h
 * @brief   IWDG 独立看门狗服务 — TIM6 ISR 喂狗策略
 * @author  CurRobo
 * @date    2026-06-05
 *
 * @note    喂狗策略: TIM6 ISR (1kHz) 中调用 watchdog_feed()
 *
 *          为什么不在 main while(1) 中喂狗?
 *            - 主循环可能被 SPI/UART/HAL_Delay 阻塞
 *            - 阻塞时 IWDG 未喂 → 误复位
 *            - TIM6 ISR 优先级最高, 不被主循环阻塞
 *            - TIM6 停了 → FDCAN MIT 帧也停了 → 电机 limp
 *              → IWDG 复位是正确的安全行为
 *
 *          调用位置:
 *            watchdog_init()   → main() 中 MX_IWDG_Init() 之后
 *            watchdog_feed()   → data_update.c 的 TIM6 ISR 末尾
 *            watchdog_check()  → main while(1) 中 (可选, 监控 ISR 是否在喂)
 *
 *          CubeMX IWDG 配置参数:
 *          ┌──────────────────────────────────────────────┐
 *          │  Pinout & Configuration → IWDG → Activate    │
 *          │                                              │
 *          │  Prescaler:         64  (500Hz @ LSI=32kHz)  │
 *          │  Window value:      0   (禁用窗口功能)        │
 *          │  Down-counter:      2000 (4秒超时)            │
 *          │                                              │
 *          │  超时计算:                                    │
 *          │    Tout = (64 / 32000) × 2000 = 4.0s         │
 *          └──────────────────────────────────────────────┘
 */
#ifndef __WATCHDOG_H__
#define __WATCHDOG_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  初始化 IWDG
 * @note   在 main() 中 MX_IWDG_Init() 之后调用.
 *         初始化后 IWDG 立即开始计数, 必须在 4s 内首次喂狗.
 */
void watchdog_init(void);

/**
 * @brief  喂狗 — 重置 IWDG 计数器
 * @note   ★ 必须在 TIM6 ISR (1kHz) 中调用, 不在主循环.
 *         TIM6 ISR 优先级最高 (NVIC prio=1), 不会被阻塞.
 *         喂狗间隔 = TIM6 周期 = 1ms, 远小于 4s 超时.
 */
void watchdog_feed(void);

/**
 * @brief  检查看门狗健康 (主循环中可选调用)
 * @note   若 TIM6 ISR 正常运行, 此函数应始终返回 0.
 *         若 > 10ms 未喂狗 → 说明 TIM6 ISR 异常 → 记录错误
 * @retval 距离上次喂狗的毫秒数 (0=正常, >10=异常)
 */
uint32_t watchdog_check(void);

#ifdef __cplusplus
}
#endif

#endif /* __WATCHDOG_H__ */
