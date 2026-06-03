/**
 * @file    wheel.c
 * @brief   轮式运动模块实现 — 遥控器摇杆 → 电机速度映射
 * @author  CurRobo
 * @date    2026-06-03
 *
 * @note    调用链路:
 *            main() while(1)
 *              → wheel_update()
 *                → 读取 remote_ctrl.rc.ch[0]
 *                → 线性映射 target_velocity = (ch1 / 660) × 1.0 rad/s
 *                → cg_ctrl_set_target(&g_cg_ctrl[0], 0, target_vel, 0)
 *
 *          MIT CAN 帧由 TIM6 ISR (1kHz) 中的 data_update_task_motor()
 *          → cg_ctrl_update_fixed() 自动发送, 本模块不直接操作 CAN.
 */
#include "wheel.h"
#include "cybergear_control.h"
#include "remote_control.h"

/* ================================================================
 *  外部引用 (main.c 定义)
 * ================================================================ */
extern CyberGear_CtrlNode_t g_cg_ctrl[];

/* ================================================================
 *  wheel_init — 初始化轮式控制
 *
 *  函数功能:
 *    将电机 0 (ID=1, FDCAN1) 配置为速度控制模式.
 *    速度控制律: τ = Kd·(ω_des − ω) + τ_ff
 *    其中 ω_des 由 wheel_update() 根据摇杆位置设定,
 *    Kd 为速度阻尼系数, τ_ff 为前馈力矩 (此处为 0).
 *
 *  速度模式 vs 阻抗模式:
 *    速度模式 (VELOCITY): 仅跟踪速度, 不跟踪位置.
 *      τ = Kd·(ω_des − ω)
 *      适合: 轮式运动、速度环控制
 *    阻抗模式 (IMPEDANCE): 同时跟踪位置和速度, 模拟弹簧-阻尼系统.
 *      τ = K·(θ_des − θ) + D·(ω_des − ω)
 *      适合: 足式站立、关节力矩控制
 *
 *  电机的 MIT 模式在硬件上电后默认为 MIT 运控模式 (CG_RUN_MIT=0),
 *  无需显式切换. 速度控制在 MCU 侧通过设定 Kp=0, Kd≠0 实现纯速度环.
 *
 *  函数参数:
 *    无 (通过外部全局变量 g_cg_ctrl[0] 访问)
 *
 *  函数输出:
 *    - g_cg_ctrl[0] 模式设为 CG_CTRL_MODE_VELOCITY
 *    - g_cg_ctrl[0] 速度阻尼设为 WHEEL_KD_VELOCITY (0.6)
 *    - 发送电机使能帧
 */
void wheel_init(void)
{
    /* 绑定电机 0: ID=1, FDCAN1 (在 pipeline_init() 中已完成初始化) */

    /* 速度模式: Kp=0 (不跟踪位置), Kd=WHEEL_KD_VELOCITY (跟踪速度) */
    cg_ctrl_set_velocity(&g_cg_ctrl[0], WHEEL_KD_VELOCITY);

    /* 初始目标速度 = 0 */
    cg_ctrl_set_target(&g_cg_ctrl[0], 0.0f, 0.0f, 0.0f);

    /* 发送使能帧 */
    cg_ctrl_enable(&g_cg_ctrl[0]);
}

/* ================================================================
 *  wheel_update — 摇杆 → 速度映射 (主循环每周期调用)
 *
 *  函数功能:
 *    读取遥控器通道 1 的当前值, 线性映射为电机目标速度.
 *
 *  映射公式:
 *    ┌─────────────────────────────────────────────────────┐
 *    │  target_velocity = (ch1 / WHEEL_RC_CH_MAX_ABS)      │
 *    │                    × WHEEL_MAX_SPEED_RAD_S           │
 *    │                                                     │
 *    │  其中:                                              │
 *    │    ch1 ∈ [-1024, +1023]  通道 1 的原始值 (中心=0)   │
 *    │    WHEEL_RC_CH_MAX_ABS = 660  (摇杆实际最大偏移)    │
 *    │    WHEEL_MAX_SPEED_RAD_S = 1.0 rad/s                │
 *    └─────────────────────────────────────────────────────┘
 *
 *  理论推导:
 *    SBUS 协议中每个通道为 11-bit (0~2047), 中位值约 1024.
 *    偏移后 ch1 = raw − 1024, 范围约 [−1024, +1023].
 *    实际摇杆机械行程限制下, 实测最大偏移约 ±660.
 *
 *    映射设计为线性比例:
 *      ω_des = (ch1 / 660) × 1.0
 *
 *    例如:
 *      ch1 =  0   → ω_des =  0.00 rad/s  (停止)
 *      ch1 = +330 → ω_des = +0.50 rad/s  (半速正转)
 *      ch1 = +660 → ω_des = +1.00 rad/s  (全速正转)
 *      ch1 = −330 → ω_des = −0.50 rad/s  (半速反转)
 *      ch1 = −660 → ω_des = −1.00 rad/s  (全速反转)
 *
 *    超出 [−660, +660] 的值会被钳位到 ±1.0 rad/s,
 *    避免因遥控器校准偏差导致的超速.
 *
 *  调用时机:
 *    主循环 while(1) 中, 每周期调用 (后台由 1kHz TIM6 ISR 发送 CAN 帧).
 *    无需在本函数中做频率控制, 因为 cg_ctrl_set_target() 仅写内存,
 *    CAN 发送由 data_update_task_motor() 按 1kHz 独立调度.
 *
 *  函数参数:
 *    无
 *
 *  函数输出:
 *    - g_cg_ctrl[0].target_velocity 被更新为摇杆映射后的目标速度
 *    - 下一个 TIM6 中断 (1ms 内) 自动发送 MIT CAN 帧到电机
 */
void wheel_update(void)
{
    /* 1. 读取遥控器通道 1 */
    int16_t ch1 = remote_ctrl.rc.ch[0];

    /* 2. 线性映射: ch1 → target_velocity (rad/s) */
    float target_velocity = (float)ch1 / (float)WHEEL_RC_CH_MAX_ABS
                            * WHEEL_MAX_SPEED_RAD_S;

    /* 3. 钳位到 [-1.0, +1.0] rad/s */
    if (target_velocity >  WHEEL_MAX_SPEED_RAD_S)
        target_velocity =  WHEEL_MAX_SPEED_RAD_S;
    if (target_velocity < -WHEEL_MAX_SPEED_RAD_S)
        target_velocity = -WHEEL_MAX_SPEED_RAD_S;

    /* 4. 更新控制目标 (仅写内存, CAN 帧由 TIM6 ISR 发送) */
    cg_ctrl_set_target(&g_cg_ctrl[0],
                       0.0f,              /* position: 速度模式不跟踪位置 */
                       target_velocity,   /* velocity: 摇杆映射的目标速度 */
                       0.0f);             /* torque_ff: 无前馈力矩 */
}
