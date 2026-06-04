/**
 * @file    motor_service.h
 * @brief   电机生命周期管理服务 — 使能重试 + 在线检测 + 故障处理
 * @author  CurRobo
 * @date    2026-06-05
 *
 * @note    本服务封装 cybergear_motor + cybergear_control,
 *          提供每个电机的独立生命周期管理和使能重试机制.
 *
 *          生命周期:
 *            MOTOR_OFFLINE → MOTOR_ENABLING → MOTOR_ONLINE
 *                                   ↑              │
 *                                   └── RETRY ── OFF← (故障/掉线)
 *
 *          调用链路:
 *            pipeline_init()  → motor_service_init()   [仅绑定, 不发 CAN]
 *            app_task_init()  → motor_service_enable_all() [首次使能]
 *            主循环            → motor_service_update()    [周期检查+重试]
 *            TIM6 ISR         → cg_ctrl_update_fixed()     [MIT 帧发送, 不变]
 */
#ifndef __MOTOR_SERVICE_H__
#define __MOTOR_SERVICE_H__

#include "main.h"
#include "cybergear_control.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  电机生命周期状态
 * ================================================================ */
typedef enum {
    MOTOR_STATE_OFFLINE  = 0,   /* 从未收到反馈 */
    MOTOR_STATE_ENABLING = 1,   /* 已发使能帧, 等待首次反馈 */
    MOTOR_STATE_ONLINE   = 2,   /* 正常在线, 受控 */
    MOTOR_STATE_FAULT    = 3    /* 故障 (过温/过流/通信超时) */
} MotorLifeState_t;

/* ================================================================
 *  电机服务对象 (在 CyberGear_CtrlNode_t 基础上扩展)
 * ================================================================ */
typedef struct {
    CyberGear_CtrlNode_t *ctrl;          /* 绑定的控制节点 */
    MotorLifeState_t      state;         /* 当前生命周期状态 */
    uint32_t              last_enable_ms;/* 上次使能时间戳 (ms) */
    uint32_t              last_online_ms;/* 上次在线时间戳 (ms) */
    uint8_t               enable_retries;/* 使能重试次数 */
    uint8_t               fault_code;    /* 最近故障码 (0=无故障) */
} MotorServiceNode_t;

/* ================================================================
 *  运动指令 (统一的控制接口, 来源可为 RC / 上位机 / 自主)
 * ================================================================ */

/** 指令来源 */
typedef enum {
    CMD_SOURCE_NONE = 0,
    CMD_SOURCE_RC,          /* 遥控器 */
    CMD_SOURCE_UPPER,       /* 上位机 (UART/CAN) */
    CMD_SOURCE_AUTO         /* 自主控制 (规划器) */
} MotionCmdSource_t;

/** 运动指令 (下位机"运控小脑"的核心输入) */
typedef struct {
    float vx;               /* 前进速度 (rad/s or m/s after scaling) */
    float vy;               /* 横向速度 */
    float wz;               /* 旋转速度 (reserved for future) */
    MotionCmdSource_t source;
    uint8_t              valid;      /* 1=指令有效 */
} MotionCommand_t;

/* ================================================================
 *  配置宏
 * ================================================================ */

/** 使能重试间隔 (ms) */
#define MOTOR_RETRY_INTERVAL_MS    500

/** 使能最大重试次数 */
#define MOTOR_MAX_RETRIES          3

/** 电机在线超时 (ms), 超过此时间未收到反馈则判定离线 */
#define MOTOR_ONLINE_TIMEOUT_MS    2000

/* ================================================================
 *  API
 * ================================================================ */

/**
 * @brief  电机服务初始化 — 绑定控制节点, 重置生命周期状态
 * @note   在 pipeline_init() 中 cg_ctrl_init 之后调用.
 *         不发送 CAN 帧, 仅初始化内存状态.
 */
void motor_service_init(void);

/**
 * @brief  使能全部电机 (首次启动)
 * @note   在 app_task_init() 中调用, 发送使能帧.
 *         若电机未上电, 后续 motor_service_update() 会自动重试.
 */
void motor_service_enable_all(void);

/**
 * @brief  电机服务周期更新 — 检查在线状态 + 自动重试
 * @note   在主循环中每周期调用.
 *         内部以 MOTOR_RETRY_INTERVAL_MS 间隔重试使能.
 * @retval 1=全部电机在线, 0=存在离线/故障
 */
uint8_t motor_service_update(void);

/**
 * @brief  获取指定电机的生命周期状态
 * @param  index  电机索引 (0~3)
 */
MotorLifeState_t motor_service_get_state(uint8_t index);

/**
 * @brief  检查是否所有电机都在线
 */
uint8_t motor_service_all_online(void);

/**
 * @brief  停止所有电机 (紧急停止, 发送 STOP 帧)
 */
void motor_service_emergency_stop(void);

/**
 * @brief  设置所有电机目标速度为零 (软停止, 不发帧)
 */
void motor_service_soft_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_SERVICE_H__ */
