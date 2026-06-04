/*
 * @Author: Yangzhi_Liu 3068126392@qq.com
 * @Date: 2026-06-05 02:29:32
 * @LastEditors: Yangzhi_Liu 3068126392@qq.com
 * @LastEditTime: 2026-06-05 02:37:08
 * @FilePath: \CurRobo\Robo\services\watchdog.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 * @file    watchdog.c
 * @brief   IWDG 独立看门狗服务实现 — TIM6 ISR 喂狗
 * @author  CurRobo
 * @date    2026-06-05
 *
 * @note    喂狗策略: TIM6 ISR (1kHz) 中调用
 *          不在主循环喂 — TIM6 ISR 优先级最高, 不被阻塞
 *
 *          若主循环阻塞 → TIM6 ISR 仍在喂狗 → 系统不误复位
 *          若 TIM6 ISR 也停了 → FDCAN MIT 帧也停了
 *            → 电机 limp → IWDG 复位是正确的安全行为
 *
 *          CubeMX IWDG 配置参数:
 *            Prescaler = 64, Window = 0, Reload = 2000
 *            超时 = (64/32000) × 2000 = 4.0s
 */
#include "watchdog.h"
#include "main.h"
#include "data_update.h"
#include "bsp_usart.h"

/* hiwdg1 由 CubeMX 生成的 iwdg.c 定义 */
extern IWDG_HandleTypeDef hiwdg1;

static volatile uint32_t g_wdg_last_feed_tick = 0;
static volatile uint32_t g_wdg_feed_count     = 0;

void watchdog_init(void)
{
    g_wdg_last_feed_tick = data_update_get_tick_ms();
    g_wdg_feed_count     = 1;
    HAL_IWDG_Refresh(&hiwdg1);
    usart1_print("[WDG] IWDG started (TIM6 ISR feed, 4s timeout)\r\n");
}

void watchdog_feed(void)
{
    HAL_IWDG_Refresh(&hiwdg1);
    g_wdg_last_feed_tick = data_update_get_tick_ms();
    g_wdg_feed_count++;
}

uint32_t watchdog_check(void)
{
    uint32_t now  = data_update_get_tick_ms();
    uint32_t last = g_wdg_last_feed_tick;
    if (now >= last) return now - last;
    return (0xFFFFFFFF - last) + now + 1;
}
