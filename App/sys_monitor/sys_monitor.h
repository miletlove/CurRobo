/**
 * @file    sys_monitor.h
 * @brief   系统联调观察模块 — 串口打印电机状态与遥控器数据
 * @author  CurRobo
 * @date    2026-10-09
 *
 * @note    联调用途:
 *           在完整生产逻辑 (pipeline + app_task + services) 运行的同时,
 *           通过 USART1 DMA 打印观察:
 *             - 电机生命周期状态 + 反馈数据 (1Hz)
 *             - 遥控器接收数据 (10Hz)
 *
 *          时基: TIM6 1kHz (data_update_get_tick_ms), 不使用 SysTick.
 *          本模块只读全局状态, 不修改控制逻辑, 不发送任何 CAN 帧.
 *
 *          集成方式:
 *            main.c 中 app_task_init() 之后调用 sys_monitor_init(),
 *            主循环 app_task_run() 之后调用 sys_monitor_run().
 *            由 SYS_MONITOR 宏控制, 联调完成后改为 0 即可关闭.
 */
#ifndef __SYS_MONITOR_H__
#define __SYS_MONITOR_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  联调观察开关: 1=打印, 0=关闭
 * ================================================================ */
#define SYS_MONITOR  1

/**
 * @brief  观察模块初始化 (打印标题)
 */
void sys_monitor_init(void);

/**
 * @brief  周期打印 (主循环每周期调用, 非阻塞)
 */
void sys_monitor_run(void);

#ifdef __cplusplus
}
#endif

#endif /* __SYS_MONITOR_H__ */
