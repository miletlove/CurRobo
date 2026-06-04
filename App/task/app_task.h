/**
 * @file    app_task.h
 * @brief   应用任务调度模块 — 启动FSM + 服务化调度 + 命令源路由
 * @author  CurRobo
 * @date    2026-06-05
 *
 * @note    启动状态机 (FSM):
 *          ┌──────────────────────────────────────────────────┐
 *          │  app_task_init()                   [main 初始化] │
 *          │    → motor_service_enable_all()    [首次使能]     │
 *          │    → can_service_init()            [CAN 监控]     │
 *          │                                                 │
 *          │  while(1) {                        [主循环]      │
 *          │    app_task_run()                               │
 *          │      → data_update_execute()       [TIM6 标志]   │
 *          │      → motor_service_update()      [重试/健康]   │
 *          │      → can_service_update()        [CAN 监控]    │
 *          │      → app_task_fsm_update()       [启动状态机]  │
 *          │      → app_task_wheel_update()     [遥控→速度]   │
 *          │      → watchdog_feed()             [看门狗]      │
 *          │  }                                              │
 *          └──────────────────────────────────────────────────┘
 *
 *          命令源路由 (为上位机预留接口):
 *            当前由遥控器驱动 (CMD_SOURCE_RC).
 *            未来可扩展 CMD_SOURCE_UPPER (串口/CAN 上位机指令).
 *            MotionCommand_t 作为统一控制接口.
 *
 *          服务化分层:
 *            App/task/app_task          ← 本模块 (FSM + 调度)
 *            Robo/services/motor_service← 电机生命周期管理
 *            Robo/services/can_service  ← CAN 总线健康监控
 *            Robo/services/watchdog     ← IWDG 看门狗
 *            Robo/wheel/wheel           ← 轮式控制 (运动学)
 *            Modules/cybergear          ← CyberGear 运控
 *            BSP/bsp_motor              ← 电机驱动 (CAN)
 *            BSP/bsp_can                ← CAN 总线 (硬件)
 */
#ifndef __APP_TASK_H__
#define __APP_TASK_H__

#include "main.h"
#include "motor_service.h"     /* MotionCommand_t 定义 */

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  启动状态机
 * ================================================================ */
typedef enum {
    APP_FSM_INIT      = 0,   /* 初始化 (首次使能已完成) */
    APP_FSM_STARTING  = 1,   /* 启动中 (等待电机上线)  */
    APP_FSM_READY     = 2,   /* 就绪   (正常控制)      */
    APP_FSM_FAULT     = 3    /* 故障   (需人为干预)    */
} AppFsmState_t;

/* ================================================================
 *  API
 * ================================================================ */

void app_task_init(void);
void app_task_run(void);
AppFsmState_t app_task_get_fsm_state(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASK_H__ */
