/*
 * @Author: Yangzhi_Liu 3068126392@qq.com
 * @Date: 2026-06-01 20:50:37
 * @LastEditors: Yangzhi_Liu 3068126392@qq.com
 * @LastEditTime: 2026-06-03 23:18:58
 * @FilePath: \CurRobo\Modules\cybergear\cybergear_control.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%A
 */
/**
 * @file    cybergear_control.h
 * @brief   CyberGear 运控接口 — 阻抗 / 速度模式封装 (精简版)
 * @author  CurRobo
 * @date    2026-06-01
 *
 * @note    在 cybergear_motor.c (底层 MIT 驱动) 之上构建:
 *          - 阻抗控制 (Impedance) : τ = K·(θ_des-θ) + D·(ω_des-ω) + τ_ff
 *          - 速度控制 (Velocity)  : τ = Kd·(ω_des-ω) + τ_ff
 *          - 空闲模式 (IDLE)      : 不下发指令
 *
 *          调用链路:
 *            cg_ctrl_set_target()    → 存目标值 (内存, 不发 CAN)
 *            cg_ctrl_set_impedance() → 设阻抗参数 + 切模式
 *            cg_ctrl_update_fixed()  → 计算 → cg_motor_mit_control() → CAN 帧
 */
#ifndef __CYBERGEAR_CONTROL_H__
#define __CYBERGEAR_CONTROL_H__

#include "cybergear_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  控制模式
 * ================================================================ */
typedef enum {
    CG_CTRL_MODE_IDLE      = 0,   /* 空闲, 不下发指令 */
    CG_CTRL_MODE_IMPEDANCE = 1,   /* 阻抗控制 */
    CG_CTRL_MODE_VELOCITY  = 2,   /* 速度控制 */
} CyberGear_CtrlMode_t;

/* ================================================================
 *  阻抗参数
 *
 *  控制律: τ = K·(θ_des - θ) + D·(ω_des - ω) + τ_ff
 *
 *  - stiffness: 刚度 K (Nm/rad), 越大越"硬"
 *  - damping:   阻尼 D (Nm/(rad/s)), 越大衰减越快
 *
 *  典型值:
 *    站立相: K=50~200,  D=2~8
 *    摆动相: K=10~50,   D=0.5~3
 *    轮式:   K=0,       D=0.1~1.0
 * ================================================================ */
typedef struct {
    float stiffness;
    float damping;
} CyberGear_Impedance_t;

/* ================================================================
 *  控制节点 (每个电机绑定一个, sizeof ≈ 44 bytes)
 * ================================================================ */
typedef struct {
    CyberGear_Motor_t   *motor;           /* 绑定的电机对象 */
    CyberGear_CtrlMode_t mode;            /* 当前控制模式 */
    float                target_position; /* 目标位置 (rad) */
    float                target_velocity; /* 目标速度 (rad/s) */
    float                target_torque;   /* 前馈力矩 (Nm) */
    CyberGear_Impedance_t impedance;      /* 阻抗参数 */
    float                kd_velocity;     /* 速度阻尼 (VELOCITY 模式用) */
    uint8_t              enabled;         /* 1=使能 */
    uint8_t              online;          /* 电机在线 */
} CyberGear_CtrlNode_t;

/* ================================================================
 *  API
 * ================================================================ */

void     cg_ctrl_init(CyberGear_CtrlNode_t *ctrl, CyberGear_Motor_t *motor);

void     cg_ctrl_set_impedance(CyberGear_CtrlNode_t *ctrl,
                               float stiffness, float damping);
void     cg_ctrl_set_velocity(CyberGear_CtrlNode_t *ctrl, float kd);
void     cg_ctrl_set_target(CyberGear_CtrlNode_t *ctrl,
                            float position, float velocity, float torque_ff);

uint8_t  cg_ctrl_enable(CyberGear_CtrlNode_t *ctrl);
uint8_t  cg_ctrl_stop(CyberGear_CtrlNode_t *ctrl);

uint8_t  cg_ctrl_update_fixed(CyberGear_CtrlNode_t *ctrl, float dt);

static inline void cg_ctrl_sync_online(CyberGear_CtrlNode_t *ctrl)
{
    if (ctrl && ctrl->motor) ctrl->online = ctrl->motor->online;
}

 /* __CYBERGEAR_CONTROL_H__ */
 /* @note   公式: τ_f = τ_coulomb · sign(ω) + b_viscous · ω
 *         库伦摩擦: 与速度方向相关, 与速度大小无关.
 *         粘滞摩擦: 与速度成正比.
 */
float cg_calc_friction_torque(const CyberGear_CtrlNode_t *ctrl, float velocity);

#endif /* __CYBERGEAR_CONTROL_H__ */
