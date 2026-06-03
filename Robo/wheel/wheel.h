/**
 * @file    wheel.h
 * @brief   轮式运动模块 — 遥控器通道→电机速度映射
 * @author  CurRobo
 * @date    2026-06-03
 *
 * @note    单电机测试用模块, 将遥控器摇杆通道线性映射为电机目标速度.
 *          摇杆通道 1 正 → 电机正转, 摇杆通道 1 负 → 电机反转.
 *          最大转速限制为 1 rad/s.
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

/** 摇杆通道最大绝对值 (SBUS 偏移后理论值 ±1024, 实际摇杆行程约 ±660) */
#define WHEEL_RC_CH_MAX_ABS    660

/** 电机最大目标转速 (rad/s) */
#define WHEEL_MAX_SPEED_RAD_S  1.0f

/** 速度阻尼系数 Kd (Nm/(rad/s)), 值越大阻尼越强, 速度跟踪越紧 */
#define WHEEL_KD_VELOCITY      0.6f

/* ================================================================
 *  API
 * ================================================================ */

/**
 * @brief  轮式模块初始化
 *
 * 函数功能:
 *   将电机 0 (ID=1, FDCAN1) 的控制模式切换为速度模式 (VELOCITY),
 *   并发送使能帧.
 *
 * 函数参数:
 *   无 (通过外部全局变量 g_cg_ctrl[] 访问控制节点)
 *
 * 函数输出:
 *   - 电机使能
 *   - 控制模式设置为 CG_CTRL_MODE_VELOCITY
 *   - 速度阻尼 Kd 设置为 WHEEL_KD_VELOCITY
 */
void wheel_init(void);

/**
 * @brief  轮式模块更新 (主循环每周期调用)
 *
 * 函数功能:
 *   读取遥控器通道 1 (remote_ctrl.rc.ch[0]) 的值,
 *   线性映射为目标速度:
 *
 *     target_velocity = (ch1 / WHEEL_RC_CH_MAX_ABS) × WHEEL_MAX_SPEED_RAD_S
 *
 *   映射公式推导:
 *     摇杆在中位时 ch1 = 0 → target_velocity = 0 (电机停转)
 *     摇杆推到底 ch1 = +660 → target_velocity = +1.0 rad/s (正转)
 *     摇杆拉到底 ch1 = -660 → target_velocity = -1.0 rad/s (反转)
 *     中间位置线性插值: 如 ch1 = +330 → target_velocity = +0.5 rad/s
 *
 *   映射后调用 cg_ctrl_set_target() 更新目标速度,
 *   实际的 MIT CAN 帧由 data_update_task_motor() 在 TIM6 ISR (1kHz) 中发送.
 *
 * 函数参数:
 *   无
 *
 * 函数输出:
 *   - 更新 g_cg_ctrl[0] 的目标速度
 *   - 电机按摇杆比例旋转
 */
void wheel_update(void);

#ifdef __cplusplus
}
#endif

#endif /* __WHEEL_H__ */
