/**
 * @file    Four.h
 * @brief   Four — 四驱差速运动模块 (普通橡胶轮, 无转向架)
 * @author  CurRobo
 * @date    2026-10-09
 *
 * @note    车辆布局 (用户提供):
 *            - ID=1, ID=2: 前轮 (FDCAN1)
 *            - ID=3, ID=4: 后轮 (FDCAN2)
 *            - ID=1, ID=3: 同侧 (左列), ID=2, ID=4: 同侧 (右列)
 *            - 无转向架, 同侧电机平行, 对侧电机相对安装
 *            - 轮胎: 普通橡胶轮 (不可横向平移)
 *
 *          差速驱动模型 (skid-steer):
 *            左列 (电机+ = 前进): v_left  = (vx + wz*GAIN) * (+1)
 *            右列 (电机+ = 后退): v_right = (vx - wz*GAIN) * (-1)
 *
 *            对侧相对安装 → 右列整体取反 (FOUR_RIGHT_DIR_SIGN = -1).
 *
 *          行为验证 (GAIN=1, 满量程=2 rad/s):
 *            全前: L=+2, R=-2 → 左右物理同向 → 全速前进
 *            全后: L=-2, R=+2 → 全速后退
 *            全右: L=+2, R=+2 → 左前右后 → 原地最大速度右转
 *            全左: L=-2, R=-2 → 原地最大速度左转
 *            斜向: 等比缩放, 单轮不超过 2 rad/s, 保持方向
 *
 *          遥控器映射 (右摇杆):
 *            ch[1] (上下): vx  前进(+)/后退(-)
 *            ch[0] (左右): wz  左转(+)/右转(-)   [差速转向, 满量程=原地旋转]
 *
 *          使能开关 (SW, s[0]):
 *            SW=UP(1)    → 电机使能 (经 motor_service 使能流程)
 *            SW=MID(3)   → 电机关闭, 不使能 (发送 STOP, 禁止自动重试)
 *            SW=DOWN(2)  → 同 MID, 关闭不使能 (安全默认)
 *
 *          注意: 开关处理在 app_task_run 中每周期执行 (任意 FSM 态生效),
 *          运动解算仅在 FSM=READY 态执行 (app_task_wheel_control).
 *
 *          速度单位: 电机轮速 rad/s, 不引入车轮半径换算,
 *          转向灵敏度由 FOUR_TURN_GAIN 调节 (板上实测).
 */
#ifndef __FOUR_H__
#define __FOUR_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  配置宏 (板上实测后按需调整)
 * ================================================================ */

/** 直行最大轮速 (rad/s) */
#define FOUR_MAX_SPEED_RAD_S      2.0f

/** 转向分量最大轮速 (rad/s), 满摇杆时左右列差速幅值 */
#define FOUR_MAX_TURN_RAD_S       2.0f

/** 转向增益: wz 分量叠加到左右列的系数 (无量纲)
 *  摇杆满偏时: v_left = -FOUR_MAX_TURN*GAIN, v_right = +... → 原地旋转 */
#define FOUR_TURN_GAIN            1.0f

/** 摇杆通道最大绝对值 (SBUS 偏移后) */
#define FOUR_RC_CH_MAX_ABS        660

/** 速度阻尼 Kd (Nm/(rad/s)) */
#define FOUR_KD_VELOCITY          0.6f

/**
 * 两侧电机安装方向符号 (对侧相对安装 → 一侧取反).
 * 板上校准: 推杆前进若车不直行/抖动, 将一侧符号取反.
 */
#define FOUR_LEFT_DIR_SIGN        (+1.0f)   /* 左列 (ID=1,3) */
#define FOUR_RIGHT_DIR_SIGN       (-1.0f)   /* 右列 (ID=2,4) */

/** 使能开关: 遥控器开关索引 (0=s[0] 左拨杆, 1=s[1] 右拨杆)
 *  DBUS 标准: s[0]=左拨杆, s[1]=右拨杆.
 *  实测: 右拨杆才有响应 → 使用 s[1]. 若需改回左拨杆, 改回 0. */
#define FOUR_ENABLE_SW_INDEX      1

/* ================================================================
 *  电机索引 (与 g_cg_ctrl[] 对应, pipeline.c 绑定事实)
 * ================================================================ */
#define FOUR_MOTOR_FL   0    /* 前, ID=1, FDCAN1, 左列 */
#define FOUR_MOTOR_FR   1    /* 前, ID=2, FDCAN1, 右列 */
#define FOUR_MOTOR_RL   2    /* 后, ID=3, FDCAN2, 左列 */
#define FOUR_MOTOR_RR   3    /* 后, ID=4, FDCAN2, 右列 */

/* ================================================================
 *  API
 * ================================================================ */

/**
 * @brief  Four 模块初始化 — 速度模式 + 零目标 + 使能帧
 * @note   在 app_task_init() 中调用 (替代原 wheel_init).
 */
void four_init(void);

/**
 * @brief  Four 模块更新 — 差速解算 (仅 FSM=READY 态调用)
 * @note   仅更新目标内存, CAN 帧由 TIM6 ISR 500Hz 自动发送.
 */
void four_update(void);

/**
 * @brief  Four 使能开关处理 (主循环每周期调用, 任意 FSM 态生效)
 * @note   sw[0]: UP=使能, MID/DOWN=关闭不使能.
 */
void four_sw_update(void);

#ifdef __cplusplus
}
#endif

#endif /* __FOUR_H__ */
