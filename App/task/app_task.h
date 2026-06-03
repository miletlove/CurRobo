/**
 * @file    app_task.h
 * @brief   应用任务调度模块 — 遥控器→电机控制工作流
 * @author  CurRobo
 * @date    2026-06-03
 *
 * @note    本模块封装了从功能初始化到任务执行的完整工作流:
 *
 *          工作流:
 *          ┌──────────────────────────────────────────────────┐
 *          │  app_task_init()                   [main 初始化] │
 *          │    → wheel_init()                  配置速度模式   │
 *          │    → 等待电机上线                               │
 *          │                                                 │
 *          │  while(1) {                        [主循环]      │
 *          │    app_task_run()                               │
 *          │      → data_update_execute()       [TIM6 标志消费]│
 *          │      → app_task_wheel_update()     [摇杆→速度映射]│
 *          │  }                                              │
 *          └──────────────────────────────────────────────────┘
 *
 *          分层关系:
 *            App/task/app_task   ← 本模块 (应用任务)
 *            Robo/wheel/wheel    ← 轮式控制 (控制算法)
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

/**
 * @brief  应用任务统一初始化
 *
 * 函数功能:
 *   在系统硬件初始化完成后, 初始化所有应用级任务:
 *     - 轮式控制初始化 (wheel_init): 配置电机为速度模式并发送使能帧
 *     - 后续可扩展: 足式步态控制、传感器处理等
 *
 * 调用时机:
 *   main() 中, pipeline_init() 之后, while(1) 之前, 仅调用一次.
 *
 * 调用顺序:
 *   main()
 *     → MX_*_Init()
 *     → pipeline_init()      [BSP 层: CAN/IMU/电机绑定]
 *     → app_task_init()      [App 层: 任务初始化]  ← 本函数
 *     → while(1) {
 *         app_task_run()     [App 层: 任务执行]    ← 见 app_task_run()
 *       }
 *
 * 函数参数:
 *   无
 *
 * 函数输出:
 *   - 电机 0 (ID=1) 已使能, 进入速度模式
 *   - 串口打印初始化日志
 */
void app_task_init(void);

/**
 * @brief  应用任务统一执行 (主循环每周期调用)
 *
 * 函数功能:
 *   按顺序执行所有应用级任务:
 *     1. data_update_execute()    — 消费 TIM6 ISR 标志 (IMU/打印/LED)
 *     2. app_task_wheel_update()  — 遥控器摇杆 → 电机速度映射
 *     3. 后续可扩展: 步态状态机、传感器数据处理等
 *
 *   各任务内部自行判断频率 (本模块的任务无需频率控制,
 *   因为 CAN 帧由 TIM6 ISR 按 1kHz 独立发送).
 *
 * 调用时机:
 *   main() while(1) 循环中, 每周期调用.
 *
 * 函数参数:
 *   无
 *
 * 函数输出:
 *   - IMU 数据更新 (200Hz)
 *   - 串口状态打印 (1Hz)
 *   - LED 刷新 (20Hz)
 *   - 电机速度随摇杆实时更新
 */
void app_task_run(void);

/**
 * @brief  轮式控制更新子任务
 *
 * 函数功能:
 *   读取遥控器通道 1, 映射为电机目标速度,
 *   调用 cg_ctrl_set_target() 更新内存中的目标值.
 *   实际的 CAN 帧由 TIM6 ISR (1kHz) 中 data_update_task_motor() 发送.
 *
 *   包含简易状态机:
 *     - 遥控器离线 → 目标速度置零 (安全保护)
 *     - 电机离线   → 目标速度置零 (安全保护)
 *     - 电机未使能 → 目标速度置零 (安全保护)
 *
 * 调用时机:
 *   由 app_task_run() 内部调用, 每主循环周期执行一次.
 *
 * 函数参数:
 *   无
 *
 * 函数输出:
 *   - g_cg_ctrl[0].target_velocity 根据摇杆位置更新
 *   - 异常情况自动置零目标速度
 */
void app_task_wheel_update(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_TASK_H__ */
