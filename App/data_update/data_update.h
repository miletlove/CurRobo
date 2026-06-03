/*
 * @Author: Yangzhi_Liu 3068126392@qq.com
 * @Date: 2026-06-03 22:16:58
 * @LastEditors: Yangzhi_Liu 3068126392@qq.com
 * @LastEditTime: 2026-06-04 01:16:36
 * @FilePath: \CurRobo\App\data_update\data_update.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 * @file    data_update.h
 * @brief   系统数据更新模块 — TIM6 1kHz 中断驱动的频率调度与数据更新
 * @author  CurRobo
 * @date    2026-06-01
 *
 * @note    架构:
 *          ─────────────────────────────────────────────────────
 *          TIM6 ISR (1kHz):
 *            HAL_TIM_PeriodElapsedCallback()
 *              → g_sys_tick_ms++                  (tick 计数器)
 *              → data_update_dispatch()            (频率调度)
 *                → data_update_task_motor()        (1kHz, 电机 CAN → ISR 安全)
 *                → data_update_task_imu()          (200Hz, 置标志)
 *                → data_update_task_print()        (1Hz,  置标志)
 *                → data_update_task_led()          (20Hz,  置标志)
 *
 *          主循环 while(1):
 *            data_update_execute()                 (消费 ISR 设置的标志)
 *              → IMU 读取     (SPI 阻塞, 不能进 ISR)
 *              → 状态打印     (UART 阻塞, 不能进 ISR)
 *              → WS2812       (时序敏感, 不能进 ISR)
 *              → 测试状态机   (含 HAL_Delay, 不能进 ISR)
 *          ─────────────────────────────────────────────────────
 */
#ifndef __DATA_UPDATE_H__
#define __DATA_UPDATE_H__

#include "main.h"
#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  任务更新频率定义 (Hz)
 * ================================================================ */
#define DATA_UPDATE_FREQ_MOTOR   500     /* 电机控制:   500Hz (4电机 CAN负载 52%) */
#define DATA_UPDATE_FREQ_IMU     200     /* IMU 读取:   200Hz */
#define DATA_UPDATE_FREQ_PRINT   1       /* 状态打印:   1Hz   */
#define DATA_UPDATE_FREQ_LED     20      /* WS2812:     20Hz  */

/* 频率 → tick 间隔换算 (tick = 1ms) */
#define DATA_UPDATE_TICK_MOTOR   (1000 / DATA_UPDATE_FREQ_MOTOR)   /* 2ms */
#define DATA_UPDATE_TICK_IMU     (1000 / DATA_UPDATE_FREQ_IMU)     /* 5    */
#define DATA_UPDATE_TICK_PRINT   (1000 / DATA_UPDATE_FREQ_PRINT)   /* 1000 */
#define DATA_UPDATE_TICK_LED     (1000 / DATA_UPDATE_FREQ_LED)     /* 50   */

/* ================================================================
 *  API 声明
 * ================================================================ */

void data_update_init(void);
uint32_t data_update_get_tick_ms(void);

/**
 * @brief  TIM6 ISR 内调用的统一频率调度入口
 * @note   由 HAL_TIM_PeriodElapsedCallback 调用 (ISR 上下文).
 *         内部根据 g_sys_tick_ms 决定各任务是否到期执行:
 *           - 电机控制 (1kHz): 直接调用 cg_ctrl_update_fixed() [ISR 安全]
 *           - IMU 读取 (200Hz): 置 g_flag_imu
 *           - 状态打印 (1Hz):   置 g_flag_print
 *           - LED 刷新 (20Hz):  置 g_flag_led
 */
void data_update_dispatch(void);

/**
 * @brief  主循环中调用的执行函数 (消费 ISR 设置的标志)
 * @note   处理所有不能在 ISR 中执行的阻塞操作:
 *           - BMI088_read()   [SPI 阻塞]
 *           - usart1_print()   [UART 阻塞]
 *           - WS2812_Rainbow() [时序敏感]
 *           - 测试状态机       [含 HAL_Delay]
 */
void data_update_execute(void);

#ifdef __cplusplus
}
#endif

#endif /* __DATA_UPDATE_H__ */
