/**
 * @file    cybergear_control.c
 * @brief   CyberGear 运控接口实现 (精简版: 阻抗 + 速度)
 * @author  CurRobo
 * @date    2026-06-01
 *
 * @note    基于 cybergear_motor.c 构建两种控制模式:
 *
 *          阻抗控制 (IMPEDANCE): τ = K·(θ_des-θ) + D·(ω_des-ω) + τ_ff
 *          速度控制 (VELOCITY):  τ = Kd·(ω_des-ω) + τ_ff
 *
 *          CAN 帧发送 (每电机):
 *            cg_ctrl_enable()        → 1 帧 (使能, 仅一次)
 *            cg_ctrl_update_fixed()  → 1 帧/周期 (1kHz MIT 控制)
 *            cg_ctrl_set_target/set_* → 0 帧 (仅存内存)
 *
 *          cg_ctrl_update_fixed() 可在 TIM6 ISR 中调用.
 *          禁止在 CAN Rx ISR 中调用以免重入.
 */
#include "cybergear_control.h"
#include <math.h>
#include <string.h>

static inline float clampf(float x, float lo, float hi)
{
    if (x > hi) return hi;
    if (x < lo) return lo;
    return x;
}

/* ---- cg_ctrl_init ---- */
void cg_ctrl_init(CyberGear_CtrlNode_t *ctrl, CyberGear_Motor_t *motor)
{
    if (!ctrl || !motor) return;
    memset(ctrl, 0, sizeof(*ctrl));
    ctrl->motor = motor;
    ctrl->mode  = CG_CTRL_MODE_IDLE;
}

/* ---- cg_ctrl_set_impedance ---- */
void cg_ctrl_set_impedance(CyberGear_CtrlNode_t *ctrl,
                           float stiffness, float damping)
{
    if (!ctrl) return;
    ctrl->mode                 = CG_CTRL_MODE_IMPEDANCE;
    ctrl->impedance.stiffness  = clampf(stiffness, 0.0f, CG_KP_MAX);
    ctrl->impedance.damping    = clampf(damping,   0.0f, CG_KD_MAX);
}

/* ---- cg_ctrl_set_velocity ---- */
void cg_ctrl_set_velocity(CyberGear_CtrlNode_t *ctrl, float kd)
{
    if (!ctrl) return;
    ctrl->mode        = CG_CTRL_MODE_VELOCITY;
    ctrl->kd_velocity = clampf(kd, 0.0f, CG_KD_MAX);
}

/* ---- cg_ctrl_set_target (仅存内存, 不发 CAN) ---- */
void cg_ctrl_set_target(CyberGear_CtrlNode_t *ctrl,
                        float position, float velocity, float torque_ff)
{
    if (!ctrl) return;
    ctrl->target_position = clampf(position, CG_P_MIN, CG_P_MAX);
    ctrl->target_velocity = clampf(velocity, CG_V_MIN, CG_V_MAX);
    ctrl->target_torque   = clampf(torque_ff, CG_T_MIN, CG_T_MAX);
}

/* ---- cg_ctrl_enable: 发送使能帧 ---- */
uint8_t cg_ctrl_enable(CyberGear_CtrlNode_t *ctrl)
{
    if (!ctrl || !ctrl->motor) return 1;
    uint8_t ret = cg_motor_enable(ctrl->motor);
    if (ret == 0) ctrl->enabled = 1;
    return ret;
}

/* ---- cg_ctrl_stop: 发送停止帧 → IDLE ---- */
uint8_t cg_ctrl_stop(CyberGear_CtrlNode_t *ctrl)
{
    if (!ctrl || !ctrl->motor) return 1;
    uint8_t ret = cg_motor_stop(ctrl->motor);
    ctrl->enabled = 0;
    ctrl->mode    = CG_CTRL_MODE_IDLE;
    return ret;
}

/* ---- cg_ctrl_update_fixed: 核心 — 计算 → MIT CAN 帧 ---- */
uint8_t cg_ctrl_update_fixed(CyberGear_CtrlNode_t *ctrl, float dt)
{
    (void)dt;

    if (!ctrl || !ctrl->motor)         return 1;
    if (ctrl->mode == CG_CTRL_MODE_IDLE) return 0;
    /* online 仅用于监控, 不阻塞控制回路.
       电机需要持续 MIT 帧才能维持反馈, 阻塞会导致死锁. */
    if (!ctrl->enabled) return 1;

    CyberGear_Motor_t *motor = ctrl->motor;
    float kp_cmd = 0.0f, kd_cmd = 0.0f;
    float pos_cmd = motor->feedback.position;
    float vel_cmd = 0.0f;
    float tor_ff  = ctrl->target_torque;

    switch (ctrl->mode)
    {
    case CG_CTRL_MODE_IMPEDANCE:
        kp_cmd  = ctrl->impedance.stiffness;
        kd_cmd  = ctrl->impedance.damping;
        pos_cmd = ctrl->target_position;
        vel_cmd = ctrl->target_velocity;
        break;

    case CG_CTRL_MODE_VELOCITY:
        kp_cmd  = 0.0f;
        kd_cmd  = ctrl->kd_velocity;
        pos_cmd = motor->feedback.position;
        vel_cmd = ctrl->target_velocity;
        break;

    default:
        return 0;
    }

    CyberGear_MITCmd_t mit;
    mit.position = clampf(pos_cmd, CG_P_MIN, CG_P_MAX);
    mit.velocity = clampf(vel_cmd, CG_V_MIN, CG_V_MAX);
    mit.torque   = clampf(tor_ff,  CG_T_MIN, CG_T_MAX);
    mit.kp       = kp_cmd;
    mit.kd       = kd_cmd;

    return cg_motor_mit_control(motor, &mit);
}