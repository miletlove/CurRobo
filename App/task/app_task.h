/**
 * @file    app_task.h
 * @brief   应用任务调度模块 — 遥控器→四轮麦克纳姆轮控制工作流
 * @author  CurRobo
 * @date    2026-06-04
 *
 * @note    本模块封装了从功能初始化到任务执行的完整工作流:
 *
 *          工作流:
 *          ┌──────────────────────────────────────────────────┐
 *          │  app_task_init()                   [main 初始化] │
 *          │    → wheel_init()                  4电机速度模式  │
 *          │    → 等待电机上线                               │
 *          │                                                 │
 *          │  while(1) {                        [主循环]      │
 *          │    app_task_run()                               │
 *          │      → data_update_execute()       [TIM6 标志消费]│
 *          │      → app_task_wheel_update()     [摇杆→四轮速度]│
 *          │  }                                              │
 *          └──────────────────────────────────────────────────┘
 *
 *          四轮麦克纳姆轮布局:
 *            FDCAN1: 电机 ID=1 (前左), ID=2 (后左)
 *            FDCAN2: 电机 ID=3 (前右), ID=4 (后右)
 *
 *          安全保护:
 *            - 遥控器离线 → 四轮全停
 *            - 任一电机未使能 → 四轮全停
 *            - 任一电机通信中断 → 四轮全停
 *
 *          分层关系:
 *            App/task/app_task   ← 本模块 (应用任务)
 *            Robo/wheel/wheel    ← 轮式控制 (运动学解算)
 *            Modules/cybergear   ← CyberGear 运控 (控制模式)
 *            BSP/bsp_motor       ← 电机驱动 (CAN 收发)
 *            BSP/bsp_can         ← CAN 总线 (硬件抽象)
 */
#ifndef __APP_TASK_H__
#define __APP_TASK_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  API
 * ================================================================ */

void app_task_init(void);
void app_task_run(void);
void app_task_wheel_update(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASK_H__ */
