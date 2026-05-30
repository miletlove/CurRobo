/**
 * @file    cybergear_motor.h
 * @brief   CyberGear 电机驱动接口 — 数据结构定义与 API 声明
 * @note    基于 CyberData.md 协议, 扩展帧 29-bit ID, MIT 控制模式
 */
#ifndef __CYBERGEAR_MOTOR_H__
#define __CYBERGEAR_MOTOR_H__

#include "main.h"
#include "fdcan.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  量化范围常量 (CyberData.md 六)
 * ================================================================ */
#define CG_P_MIN   (-12.56637061f)   /* -4 rad       */
#define CG_P_MAX   ( 12.56637061f)   /* +4 rad       */
#define CG_V_MIN   (-30.0f)          /* -30 rad/s     */
#define CG_V_MAX   ( 30.0f)          /* +30 rad/s     */
#define CG_T_MIN   (-12.0f)          /* -12 Nm        */
#define CG_T_MAX   ( 12.0f)          /* +12 Nm        */
#define CG_KP_MIN  (0.0f)            /* 0             */
#define CG_KP_MAX  (500.0f)          /* 500           */
#define CG_KD_MIN  (0.0f)            /* 0             */
#define CG_KD_MAX  (5.0f)            /* 5             */

/* ================================================================
 *  通信类型 (扩展 ID bit24-28 的 mode 字段, CyberData.md 三)
 * ================================================================ */
typedef enum {
    CG_TYPE_GET_ID      = 0,   /* 获取设备 ID (广播)      */
    CG_TYPE_MIT_CTRL    = 1,   /* MIT 运控模式            */
    CG_TYPE_FEEDBACK    = 2,   /* 电机反馈帧              */
    CG_TYPE_ENABLE      = 3,   /* 电机使能                */
    CG_TYPE_STOP        = 4,   /* 电机停止                */
    CG_TYPE_SET_ZERO    = 6,   /* 设置机械零位            */
    CG_TYPE_SET_ID      = 7,   /* 设置 CAN_ID             */
    CG_TYPE_READ_PARAM  = 17,  /* 参数读取                */
    CG_TYPE_WRITE_PARAM = 18,  /* 参数写入                */
    CG_TYPE_FAULT       = 21   /* 故障反馈                */
} CyberGear_Type_t;

/* ================================================================
 *  电机运行模式 (CyberData.md 十)
 * ================================================================ */
typedef enum {
    CG_RUN_MIT   = 0,   /* MIT 运控模式 (上电默认) */
    CG_RUN_POS   = 1,   /* 位置模式                */
    CG_RUN_SPEED = 2,   /* 速度模式                */
    CG_RUN_CUR   = 3    /* 电流模式                */
} CyberGear_RunMode_t;

/* ================================================================
 *  电机模式状态 (反馈帧中, CyberData.md 八)
 * ================================================================ */
typedef enum {
    CG_STATE_RESET = 0,
    CG_STATE_CALI  = 1,
    CG_STATE_MOTOR = 2
} CyberGear_State_t;

/* ================================================================
 *  数据结构
 * ================================================================ */

/** MIT 控制命令 (CyberData.md 十二) */
typedef struct {
    float position;   /* 目标位置 (rad)   [-4, +4]    */
    float velocity;   /* 目标速度 (rad/s) [-30, +30]    */
    float torque;     /* 前馈力矩 (Nm)    [-12, +12]    */
    float kp;         /* 位置环增益       [0, 500]      */
    float kd;         /* 速度环增益       [0, 5]        */
} CyberGear_MITCmd_t;

/** 电机反馈信息 (CyberData.md 八) */
typedef struct {
    float    position;     /* 当前角度 (rad)         */
    float    velocity;     /* 当前速度 (rad/s)       */
    float    torque;       /* 当前力矩 (Nm)          */
    float    temperature;  /* 当前温度 ()           */
    uint8_t  mode_state;   /* 模式状态 (0/1/2)       */
    uint8_t  fault;        /* 故障码 (bit 组合)      */
} CyberGear_Feedback_t;

/** 电机对象 */
typedef struct {
    uint8_t               motor_id;    /* 电机 CAN ID (0~127)        */
    FDCAN_HandleTypeDef  *hcan;        /* 所属 FDCAN 总线句柄        */
    CyberGear_Feedback_t  feedback;    /* 最新反馈数据               */
    uint8_t               online;      /* 在线标志 (收到反馈则置1)    */
} CyberGear_Motor_t;

/* ================================================================
 *  API 声明
 * ================================================================ */

/* --- 生命周期 --- */
void     cg_motor_init(CyberGear_Motor_t *motor, uint8_t id,
                       FDCAN_HandleTypeDef *hcan);
uint8_t  cg_motor_enable(CyberGear_Motor_t *motor);
uint8_t  cg_motor_stop(CyberGear_Motor_t *motor);
uint8_t  cg_motor_set_zero(CyberGear_Motor_t *motor);

/* --- MIT 控制 --- */
uint8_t  cg_motor_mit_control(CyberGear_Motor_t *motor,
                              const CyberGear_MITCmd_t *cmd);

/* --- 运行模式切换 --- */
uint8_t  cg_motor_set_run_mode(CyberGear_Motor_t *motor,
                               CyberGear_RunMode_t mode);

/* --- 反馈解析 (在 CAN 中断回调中调用) --- */
void     cg_motor_parse_feedback(uint32_t ext_id, const uint8_t *data,
                                 CyberGear_Feedback_t *fb);

/* --- 量化工具函数 --- */
uint16_t cg_float_to_uint(float x, float x_min, float x_max);
float    cg_uint_to_float(uint16_t x, float x_min, float x_max);

/* --- 反馈回调 (弱符号, 应用层可在 main.c 中重写) --- */
/**
 * @brief  电机反馈通知回调 — 在 CAN 中断上下文中调用, 需保持简短
 * @param  motor  收到反馈的电机对象指针
 * @note   默认空实现. 应用层重写后可做: 故障检测/数据记录/状态切换等.
 *         重写时避免调用阻塞函数 (HAL_Delay 等) 或重 CAN 操作.
 */
void cg_motor_on_feedback(CyberGear_Motor_t *motor);

#ifdef __cplusplus
}
#endif

#endif /* __CYBERGEAR_MOTOR_H__ */
