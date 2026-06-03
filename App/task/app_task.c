/**
 * @file    app_task.c
 * @brief   应用任务调度模块实现 — 遥控器→电机控制工作流
 * @author  CurRobo
 * @date    2026-06-03
 *
 * @note    本模块是应用层入口, 连接 BSP 层 (电机/CAN) 和 Robo 层 (控制算法).
 *
 *          控制流:
 *          ┌─────────────┐
 *          │ HW Init     │  MX_*_Init(), SystemClock_Config()
 *          ├─────────────┤
 *          │ pipeline    │  CAN/IMU/电机绑定/TIM6 启动
 *          ├─────────────┤
 *          │ app_task    │  wheel_init() → 电机使能 + 速度模式
 *          ├─────────────┤
 *          │ while(1)    │
 *          │  app_task   │  data_update_execute() + wheel_update()
 *          │  run()      │  ← 每周期
 *          └─────────────┘
 *
 *          实时性保证:
 *            - CAN 帧: TIM6 ISR (1kHz, 最高优先级)
 *            - 摇杆读取: 主循环 (非 ISR, 但遥控器数据由 DBUS ISR 以 ~100Hz 更新)
 *            - IMU 读取: 主循环 (200Hz 标志触发, SPI 阻塞)
 *            - 串口打印: 主循环 (1Hz 标志触发)
 */
#include "app_task.h"
#include "data_update.h"
#include "wheel.h"
#include "cybergear_control.h"
#include "remote_control.h"
#include "bsp_usart.h"

/* ================================================================
 *  外部引用 (main.c 定义)
 * ================================================================ */
extern CyberGear_CtrlNode_t g_cg_ctrl[];

/* ================================================================
 *  内部状态
 * ================================================================ */

/** 轮式控制在线状态标志 */
static uint8_t wheel_was_online = 0;

/* ================================================================
 *  app_task_init — 应用任务统一初始化
 *
 *  函数功能:
 *    在 pipeline_init() 完成 BSP 层初始化后,
 *    初始化所有应用级任务:
 *      - wheel_init(): 电机 0 切换为速度模式, 发送使能帧
 *
 *  函数参数:
 *    无
 *
 *  函数输出:
 *    - 电机 0 使能并进入速度模式
 *    - 串口打印初始化日志
 */
void app_task_init(void)
{
    usart1_print("\r\n===== App Task Init =====\r\n");

    /* ---- 轮式控制初始化 ---- */
    usart1_print("Wheel control init... ");
    wheel_init();
    usart1_print("OK (SPEED mode, Kd=%.1f, max=%.1f rad/s)\r\n",
                 WHEEL_KD_VELOCITY, WHEEL_MAX_SPEED_RAD_S);

    usart1_print("==========================\r\n");
}

/* ================================================================
 *  app_task_run — 应用任务统一执行
 *
 *  函数功能:
 *    每周期在 main() while(1) 中调用, 按顺序执行:
 *      1. data_update_execute()    消费 TIM6 ISR 标志
 *      2. app_task_wheel_update()  遥控器→电机速度映射
 *
 *  函数参数:
 *    无
 *
 *  函数输出:
 *    - 各周期任务按标志消费执行
 *    - 电机速度根据摇杆实时更新
 */
void app_task_run(void)
{
    /* ---- 1. TIM6 ISR 标志消费 ---- */
    data_update_execute();

    /* ---- 2. 轮式控制更新 ---- */
    app_task_wheel_update();
}

/* ================================================================
 *  app_task_wheel_update — 轮式控制更新 (含安全保护)
 *
 *  函数功能:
 *    从遥控器读取通道 1 的值, 映射为电机目标速度.
 *    包含三层安全保护:
 *      1. 遥控器离线 → 目标速度置零
 *      2. 电机离线   → 目标速度置零
 *      3. 电机未使能 → 目标速度置零
 *
 *  安全设计:
 *    当遥控器信号丢失 (remote_ctrl.online==0) 或电机通信中断时,
 *    自动将目标速度置零, 电机在速度阻尼 Kd 作用下快速停止.
 *    本函数写入的 target_velocity 在下一个 TIM6 中断 (≤1ms) 即生效.
 *
 *  函数参数:
 *    无
 *
 *  函数输出:
 *    - g_cg_ctrl[0].target_velocity 更新
 *    - 异常状态串口提示 (仅状态变化时打印, 避免刷屏)
 */
void app_task_wheel_update(void)
{
    /* ---------- 安全保护 1: 遥控器离线 ---------- */
    if (!remote_ctrl.online)
    {
        cg_ctrl_set_target(&g_cg_ctrl[0], 0.0f, 0.0f, 0.0f);
        wheel_was_online = 0;
        return;
    }

    /* ---------- 安全保护 2: 电机离线 ---------- */
    if (!g_cg_ctrl[0].online)
    {
        cg_ctrl_set_target(&g_cg_ctrl[0], 0.0f, 0.0f, 0.0f);
        wheel_was_online = 0;
        return;
    }

    /* ---------- 安全保护 3: 电机未使能 ---------- */
    if (!g_cg_ctrl[0].enabled)
    {
        cg_ctrl_set_target(&g_cg_ctrl[0], 0.0f, 0.0f, 0.0f);
        return;
    }

    /* ---------- 正常: 摇杆 → 速度映射 ---------- */
    wheel_was_online = 1;
    wheel_update();
}
