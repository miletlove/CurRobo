/**
 * @file    wheel.h
 * @brief   四轮麦克纳姆轮运动控制模块 (X型布局)
 * @author  CurRobo
 * @date    2026-06-04
 *
 * @note    硬件连接:
 *          - 电机 1 (FL, 前左): FDCAN1, ID=1
 *          - 电机 2 (RL, 后左): FDCAN1, ID=2
 *          - 电机 3 (FR, 前右): FDCAN2, ID=3
 *          - 电机 4 (RR, 后右): FDCAN2, ID=4
 *
 *          麦克纳姆轮 X 型布局逆运动学 (纯平移):
 *            v_fl = vx - vy    (前左, ID=1)
 *            v_rl = vx + vy    (后左, ID=2)
 *            v_fr = vx + vy    (前右, ID=3)
 *            v_rr = vx - vy    (后右, ID=4)
 *
 *          遥控器映射:
 *            通道 0 (ch1): 前进/后退 (vx)
 *            通道 1 (ch2): 左移/右移 (vy)
 *
 *          速度方向约定:
 *            vx > 0: 前进,  vx < 0: 后退
 *            vy > 0: 左移,  vy < 0: 右移
 */
#ifndef __WHEEL_H__
#define __WHEEL_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  配置宏
 * ================================================================ */

/** 电机最大目标转速 (rad/s), 单轮限幅 */
#define WHEEL_MAX_SPEED_RAD_S    2.0f

/** 摇杆通道最大绝对值 (SBUS 偏移后, 实际行程约 ±660) */
#define WHEEL_RC_CH_MAX_ABS      660

/** 速度阻尼系数 Kd (Nm/(rad/s)), 值越大速度跟踪越紧 */
#define WHEEL_KD_VELOCITY        0.6f

/* ================================================================
 *  电机索引 (与 g_cg_ctrl[] 数组索引对应)
 * ================================================================ */
#define WHEEL_MOTOR_FL    0    /* 前左, ID=1, FDCAN1 */
#define WHEEL_MOTOR_RL    1    /* 后左, ID=2, FDCAN1 */
#define WHEEL_MOTOR_FR    2    /* 前右, ID=3, FDCAN2 */
#define WHEEL_MOTOR_RR    3    /* 后右, ID=4, FDCAN2 */
#define WHEEL_MOTOR_COUNT 4

/* ================================================================
 *  API
 * ================================================================ */

/**
 * @brief  四轮模块初始化
 *
 * 函数功能:
 *   将全部 4 个电机配置为速度控制模式 (VELOCITY),
 *   初始化目标速度为零, 并发送使能帧.
 *
 *   速度控制律: τ = Kd·(ω_des − ω) + τ_ff
 *
 * 调用时机:
 *   app_task_init() 中调用, 在 pipeline_init() 之后, while(1) 之前.
 *
 * 函数参数:
 *   无 (通过外部全局变量 g_cg_ctrl[] 访问控制节点)
 *
 * 函数输出:
 *   - 4 个电机全部使能
 *   - 控制模式设置为 CG_CTRL_MODE_VELOCITY
 *   - 速度阻尼 Kd = WHEEL_KD_VELOCITY
 */
void wheel_init(void);

/**
 * @brief  四轮模块更新 (主循环每周期调用)
 *
 * 函数功能:
 *   读取遥控器通道 0 (前进/后退) 和通道 1 (左移/右移),
 *   通过 X 型麦克纳姆轮逆运动学解算出各轮目标速度,
 *   调用 cg_ctrl_set_target() 更新各电机控制目标.
 *
 *   实际 CAN 帧由 TIM6 ISR (500Hz) 中的 data_update_task_motor()
 *   → cg_ctrl_update_fixed() 自动发送.
 *
 * 安全钳位:
 *   当合成速度超出单轮限幅 WHEEL_MAX_SPEED_RAD_S 时,
 *   等比缩放所有车轮速度, 保持运动方向不变.
 *
 * 调用时机:
 *   主循环 while(1) → app_task_run() → app_task_wheel_update()
 *   → wheel_update() 每周期调用.
 *
 * 函数参数:
 *   无
 *
 * 函数输出:
 *   - g_cg_ctrl[0..3].target_velocity 被更新
 *   - 下一个 TIM6 中断 (2ms 内) 自动发送 MIT CAN 帧
 */
void wheel_update(void);

#ifdef __cplusplus
}
#endif

#endif /* __WHEEL_H__ */
