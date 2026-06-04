/**
 * @file    app_task.c
 * @brief   应用任务调度模块实现 — 遥控器→四轮麦克纳姆轮控制工作流
 * @author  CurRobo
 * @date    2026-06-04
 *
 * @note    本模块是应用层入口, 连接 BSP 层 (电机/CAN) 和 Robo 层 (控制算法).
 *
 *          控制流:
 *          ┌─────────────┐
 *          │ HW Init     │  MX_*_Init(), SystemClock_Config()
 *          ├─────────────┤
 *          │ pipeline    │  CAN/IMU/电机绑定/TIM6 启动
 *          ├─────────────┤
 *          │ app_task    │  wheel_init() → 4电机使能 + 速度模式
 *          ├─────────────┤
 *          │ while(1)    │
 *          │  app_task   │  data_update_execute() + wheel_update()
 *          │  run()      │  ← 每周期
 *          └─────────────┘
 *
 *          四轮控制:
 *            FDCAN1: 电机 ID=1 (前左), ID=2 (后左)
 *            FDCAN2: 电机 ID=3 (前右), ID=4 (后右)
 *
 *            X型麦克纳姆轮逆运动学:
 *              v_fl = vx - vy    v_rl = vx + vy
 *              v_fr = vx + vy    v_rr = vx - vy
 *
 *          实时性保证:
 *            - CAN 帧: TIM6 ISR (500Hz, 最高优先级)
 *            - 摇杆读取: 主循环 (非 ISR, 但遥控器数据由 DBUS ISR 以 ~100Hz 更新)
 *            - IMU 读取: 主循环 (200Hz 标志触发, SPI 阻塞)
 *            - 串口打印: 主循环 (1Hz 标志触发)
 */
#include "app_task.h"
#include "data_update.h"
#include "wheel.h"
#include "cybergear_control.h"
#include "remote_control.h"

/* ================================================================
 *  外部引用 (main.c 定义)
 * ================================================================ */
extern CyberGear_CtrlNode_t g_cg_ctrl[];

/* ================================================================
 *  内部状态
 * ================================================================ */

/** 四轮健康状态: 所有电机在线且使能 */
static uint8_t wheel_all_healthy = 0;

/* ================================================================
 *  内部辅助: 停止所有电机
 * ================================================================ */
static void wheel_stop_all(void)
{
    for (uint8_t i = 0; i < WHEEL_MOTOR_COUNT; i++)
    {
        cg_ctrl_set_target(&g_cg_ctrl[i], 0.0f, 0.0f, 0.0f);
    }
}

/* ================================================================
 *  app_task_init — 应用任务统一初始化
 * ================================================================ */
void app_task_init(void)
{
    /* 四轮麦克纳姆轮控制初始化 */
    wheel_init();
}

/* ================================================================
 *  app_task_run — 应用任务统一执行
 * ================================================================ */
void app_task_run(void)
{
    data_update_execute();
    app_task_wheel_update();
}

/* ================================================================
 *  app_task_wheel_update — 四轮控制更新 (含安全保护)
 *
 *  安全设计:
 *    三层保护机制, 任一条件不满足则立即将所有电机目标速度置零:
 *      1. 遥控器离线 → 全部置零
 *      2. 任一电机未使能 → 全部置零
 *      3. 任一电机离线 (通信中断) → 全部置零
 *
 *    四轮必须同步停止, 防止单轮失控导致车体旋转或侧翻.
 *
 *  健康状态转换:
 *    wheel_all_healthy 在全部电机在线且使能时置 1,
 *    在任一异常时置 0. 状态切换时通过串口打印日志.
 * ================================================================ */
void app_task_wheel_update(void)
{
    /* ---------- 安全保护 1: 遥控器离线 ---------- */
    if (!remote_ctrl.online)
    {
        wheel_stop_all();
        if (wheel_all_healthy)
        {
            wheel_all_healthy = 0;
            // LOG_WARN("WHEEL", "Remote offline, all motors stopped");
        }
        return;
    }

    /* ---------- 安全保护 2: 检查所有电机使能状态 ---------- */
    for (uint8_t i = 0; i < WHEEL_MOTOR_COUNT; i++)
    {
        if (!g_cg_ctrl[i].enabled)
        {
            wheel_stop_all();
            if (wheel_all_healthy)
            {
                wheel_all_healthy = 0;
                // LOG_WARN("WHEEL", "Motor %d not enabled, all stopped", i + 1);
            }
            return;
        }
    }

    /* ---------- 安全保护 3: 检查所有电机在线 ---------- */
    for (uint8_t i = 0; i < WHEEL_MOTOR_COUNT; i++)
    {
        if (!g_cg_ctrl[i].online)
        {
            wheel_stop_all();
            if (wheel_all_healthy)
            {
                wheel_all_healthy = 0;
                // LOG_WARN("WHEEL", "Motor %d offline, all stopped", i + 1);
            }
            return;
        }
    }

    /* ---------- 所有安全检查通过 ---------- */
    if (!wheel_all_healthy)
    {
        wheel_all_healthy = 1;
        // LOG_INFO("WHEEL", "All 4 motors healthy, control active");
    }

    /* 正常: 摇杆 → 四轮速度映射 */
    wheel_update();
}
