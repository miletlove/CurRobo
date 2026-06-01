/**
 * @file    cybergear_control.h
 * @brief   CyberGear 高级运控接口 — 面向机器人关节控制的模式封装
 * @author  CurRobo
 * @date    2026-06-01
 *
 * @note    在 cybergear_motor.c (底层 MIT 驱动) 之上构建:
 *          - 阻抗控制 (Impedance Control)
 *          - 位置控制 (Position PID)
 *          - 速度控制 (Velocity Control)
 *          - 力矩控制 (Torque Control)
 *          - 轨迹跟踪 (Trajectory Tracking)
 *          - 重力补偿 (Gravity Compensation)
 *          - 摩擦补偿 (Friction Compensation)
 *
 *          所有控制模式最终通过 cg_ctrl_send() 下发 MIT 帧.
 *          调用者 (应用层状态机) 选择模式 → 设参 → cg_ctrl_update() → 自动下发.
 */
#ifndef __CYBERGEAR_CONTROL_H__
#define __CYBERGEAR_CONTROL_H__

#include "cybergear_motor.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  控制模式枚举
 * ================================================================ */
typedef enum {
    CG_CTRL_MODE_IDLE       = 0,   /* 空闲 (不下发指令)               */
    CG_CTRL_MODE_IMPEDANCE  = 1,   /* 阻抗控制                        */
    CG_CTRL_MODE_POSITION   = 2,   /* 位置 PID 控制                   */
    CG_CTRL_MODE_VELOCITY   = 3,   /* 速度控制                        */
    CG_CTRL_MODE_TORQUE     = 4,   /* 力矩 (开环) 控制                 */
    CG_CTRL_MODE_TRAJECTORY = 5,   /* 轨迹跟踪 (多项式插值)             */
} CyberGear_CtrlMode_t;

/* ================================================================
 *  阻抗参数结构体
 *
 *  阻抗控制律 (Impedance Control Law):
 *
 *      τ = K_stiff · (θ_des - θ) + D_damp · (ω_des - ω) + τ_ff
 *
 *  物理意义:
 *    - K_stiff (刚度): 类似于弹簧, 单位 Nm/rad. 越大 "越硬".
 *    - D_damp  (阻尼): 类似于阻尼器, 单位 Nm/(rad/s). 越大 振荡衰减越快.
 *    - inertia (惯量): 用于导纳控制 (可选), 单位 Nm/(rad/s²).
 *
 *  典型取值 (四足机器人):
 *    - 站立相: K=50~200,  D=2~8   (刚性支撑)
 *    - 摆动相: K=10~50,   D=0.5~3 (柔顺摆动)
 * ================================================================ */
typedef struct {
    float stiffness;      /* K: 刚度 (Nm/rad)                    */
    float damping;        /* D: 阻尼 (Nm/(rad/s))               */
    float inertia;        /* M: 惯量 (Nm/(rad/s²)), 暂预留       */
} CyberGear_Impedance_t;

/* ================================================================
 *  位置 PID 参数结构体
 *
 *  PID 控制律:
 *
 *      e = θ_des - θ
 *      τ = Kp·e + Ki·∫e·dt + Kd·(de/dt) + τ_ff
 *
 *  注意: MIT 模式原生提供 Kp/Kd, 此处额外提供 Ki 积分项.
 *        实际下发时: MIT.KP = Ctrl.Kp, MIT.KD = Ctrl.Kd.
 * ================================================================ */
typedef struct {
    float kp;             /* 比例增益 (Nm/rad)                  */
    float ki;             /* 积分增益 (Nm/(rad·s))              */
    float kd;             /* 微分增益 (Nm/(rad/s))              */
    float integral_limit; /* 积分限幅 (Nm), 防积分饱和           */
    float max_torque;     /* 输出力矩限幅 (Nm)                  */
} CyberGear_PID_t;

/* ================================================================
 *  轨迹点结构体
 *
 *  轨迹由若干个轨迹点组成, 每个点定义:
 *    - 目标位置/速度/前馈力矩
 *    - 到达该点的持续时间
 *
 *  插值方式: 五次多项式 (最小 Jerk 轨迹)
 *
 *      归一化时间 s = t / T ∈ [0, 1]
 *      θ(s) = θ_0 + (θ_f - θ_0) · (10s³ - 15s⁴ + 6s⁵)
 *      ω(s) = (θ_f - θ_0) · (30s² - 60s³ + 30s⁴) / T
 *      α(s) = (θ_f - θ_0) · (60s - 180s² + 120s³) / T²
 * ================================================================ */
typedef struct {
    float    position;     /* 目标位置 (rad)                     */
    float    velocity;     /* 目标速度 (rad/s)                   */
    float    torque_ff;    /* 前馈力矩 (Nm)                      */
    uint32_t duration_ms;  /* 从上一个点到此点的耗时 (ms)         */
} CyberGear_TrajPoint_t;

/* ================================================================
 *  轨迹状态 (内部用, 跟踪插值进度)
 * ================================================================ */
typedef struct {
    CyberGear_TrajPoint_t start;      /* 起始点                    */
    CyberGear_TrajPoint_t target;     /* 目标点                    */
    uint32_t              elapsed_ms; /* 已运行时间 (ms)           */
    uint8_t               active;     /* 轨迹是否进行中            */
} CyberGear_TrajState_t;

/* ================================================================
 *  单关节控制上下文 (每个电机绑定一个)
 *
 *  将电机对象 + 控制参数 + 状态封装为一个控制节点.
 *  应用层维护一个包含 8 个节点的数组 g_ctrl[].
 * ================================================================ */
typedef struct {
    /* --- 绑定的电机对象 --- */
    CyberGear_Motor_t *motor;          /* 指向电机对象 (不可为空)   */

    /* --- 当前控制模式 --- */
    CyberGear_CtrlMode_t mode;         /* 当前控制模式             */

    /* --- 控制目标 (根据 mode 不同, 使用不同字段) --- */
    float target_position;             /* 目标位置 (rad)           */
    float target_velocity;             /* 目标速度 (rad/s)         */
    float target_torque;               /* 目标力矩 (Nm)            */

    /* --- 控制参数 --- */
    CyberGear_Impedance_t impedance;   /* 阻抗控制参数             */
    CyberGear_PID_t       pid;         /* PID 控制参数             */

    /* --- PID 积分项 --- */
    float integral_error;              /* 累积积分误差             */

    /* --- 重力/摩擦补偿 --- */
    float mass_kg;                     /* 连杆质量 (kg)            */
    float com_length_m;                /* 质心到转轴距离 (m)       */
    float coulomb_friction;            /* 库伦摩擦 (Nm)           */
    float viscous_friction;            /* 粘滞摩擦系数 (Nm/(rad/s)) */

    /* --- 轨迹状态 --- */
    CyberGear_TrajState_t traj;        /* 轨迹插值状态             */

    /* --- 上一次更新时刻 (用于 dt 计算) --- */
    uint32_t last_update_tick;         /* data_update_get_tick_ms() 时间戳 */

    /* --- 使能标志 --- */
    uint8_t enabled;                   /* 1=使能, 0=去使能         */
    uint8_t online;                    /* 电机在线 (由 feedback 更新) */
} CyberGear_CtrlNode_t;

/* ================================================================
 *  控制输出结构体 (传给 MIT 接口)
 * ================================================================ */
typedef struct {
    float position;
    float velocity;
    float torque_ff;
    float kp;
    float kd;
} CyberGear_CtrlOut_t;

/* ================================================================
 *  API 声明
 * ================================================================ */

/* --- 控制节点初始化 --- */

/**
 * @brief  将电机对象绑定到控制节点, 并设置默认参数
 * @param  ctrl    控制节点指针
 * @param  motor   已初始化的电机对象指针
 * @note   初始化后默认 mode=IDLE, 需要调用 cg_ctrl_set_mode() 切换模式.
 */
void cg_ctrl_init(CyberGear_CtrlNode_t *ctrl, CyberGear_Motor_t *motor);

/* --- 模式切换与参数设置 --- */

/**
 * @brief  切换到阻抗控制模式
 * @param  ctrl        控制节点
 * @param  stiffness   刚度 K (Nm/rad)
 * @param  damping     阻尼 D (Nm/(rad/s))
 */
void cg_ctrl_set_impedance(CyberGear_CtrlNode_t *ctrl,
                           float stiffness, float damping);

/**
 * @brief  切换到位置 PID 控制模式
 * @param  ctrl  控制节点
 * @param  kp    比例增益 (Nm/rad)
 * @param  ki    积分增益 (Nm/(rad·s)), 传 0 则退化为 PD 控制
 * @param  kd    微分增益 (Nm/(rad/s))
 */
void cg_ctrl_set_position_pid(CyberGear_CtrlNode_t *ctrl,
                              float kp, float ki, float kd);

/**
 * @brief  切换到速度控制模式
 * @param  ctrl  控制节点
 * @param  kd    速度阻尼增益 (Nm/(rad/s)), 越大跟踪越快
 */
void cg_ctrl_set_velocity(CyberGear_CtrlNode_t *ctrl, float kd);

/**
 * @brief  切换到力矩 (开环) 控制模式
 * @param  ctrl  控制节点
 */
void cg_ctrl_set_torque(CyberGear_CtrlNode_t *ctrl);

/* --- 目标值设定 --- */

/**
 * @brief  设定控制目标
 * @param  ctrl      控制节点
 * @param  position  目标位置 (rad), 仅 POSITION/IMPEDANCE 模式有效
 * @param  velocity  目标速度 (rad/s), 仅 VELOCITY/IMPEDANCE 模式有效
 * @param  torque_ff 前馈力矩 (Nm), 所有模式均有效
 */
void cg_ctrl_set_target(CyberGear_CtrlNode_t *ctrl,
                        float position, float velocity, float torque_ff);

/* --- 轨迹接口 --- */

/**
 * @brief  启动轨迹跟踪 (阻塞式设定)
 * @param  ctrl    控制节点
 * @param  points  轨迹点数组
 * @param  count   轨迹点数量
 * @note   调用后 ctrl->mode 自动切换为 TRAJECTORY,
 *         每个周期调用 cg_ctrl_update() 自动推进.
 */
void cg_ctrl_start_trajectory(CyberGear_CtrlNode_t *ctrl,
                              const CyberGear_TrajPoint_t *points,
                              uint8_t count);

/* --- 补偿参数设置 --- */

/**
 * @brief  设置重力补偿参数
 * @param  ctrl         控制节点
 * @param  mass_kg      连杆质量 (kg)
 * @param  com_length_m 质心到转轴距离 (m)
 * @note   重力补偿力矩 = mass_kg · g · com_length_m · sin(position)
 *         自动叠加到每个周期的前馈力矩中.
 */
void cg_ctrl_set_gravity_comp(CyberGear_CtrlNode_t *ctrl,
                              float mass_kg, float com_length_m);

/**
 * @brief  设置摩擦补偿参数
 * @param  ctrl        控制节点
 * @param  coulomb     库伦摩擦 (Nm)
 * @param  viscous     粘滞摩擦系数 (Nm/(rad/s))
 * @note   摩擦补偿力矩 = coulomb · sign(velocity) + viscous · velocity
 *         自动叠加到每个周期的前馈力矩中.
 */
void cg_ctrl_set_friction_comp(CyberGear_CtrlNode_t *ctrl,
                               float coulomb, float viscous);

/* --- 核心更新函数 --- */

/**
 * @brief  每个控制周期调用一次 — 计算控制量并通过 CAN 下发
 * @param  ctrl  控制节点
 * @return 0=成功, 1=电机不在线/未使能/发送失败
 *
 * @note   调用频率建议 1kHz (与 MIT 模式匹配).
 *         在 FreeRTOS 任务中调用, 或在定时器中断中调用.
 *         内部自动检查 online 标志, 电机离线则跳过.
 */
uint8_t cg_ctrl_update(CyberGear_CtrlNode_t *ctrl);

/**
 * @brief  固定 dt 版本 — 适用于 TIM6 1kHz ISR 等固定频率场景
 * @param  ctrl  控制节点
 * @param  dt    控制周期 (秒), TIM6 1kHz 时固定传 0.001f
 * @return 0=成功, 1=失败
 * @note   本函数不调用 data_update_get_tick_ms(), dt 由调用方提供.
 *         内部通过 cg_motor_mit_control() → HAL_FDCAN_AddMessageToTxFifoQ()
 *         发送 CAN 帧, STM32H7 上该 HAL 函数是 ISR 安全的.
 */
uint8_t cg_ctrl_update_fixed(CyberGear_CtrlNode_t *ctrl, float dt);

/**
 * @brief  电机使能 (带控制参数初始化)
 * @param  ctrl  控制节点
 * @return 0=成功
 */
uint8_t cg_ctrl_enable(CyberGear_CtrlNode_t *ctrl);

/**
 * @brief  电机停止 (去使能)
 * @param  ctrl  控制节点
 * @return 0=成功
 */
uint8_t cg_ctrl_stop(CyberGear_CtrlNode_t *ctrl);

/* --- 在线状态同步 (在 CAN 回调中调用) --- */

/**
 * @brief  将反馈帧中的在线状态同步到控制节点
 * @param  ctrl  控制节点
 * @note   在 cg_motor_on_feedback() 回调中调用:
 *         ctrl->online = ctrl->motor->online;
 */
static inline void cg_ctrl_sync_online(CyberGear_CtrlNode_t *ctrl)
{
    if (ctrl && ctrl->motor)
    {
        ctrl->online = ctrl->motor->online;
    }
}

/* --- 内部辅助: 重力 / 摩擦补偿计算 --- */

/**
 * @brief  计算重力补偿力矩
 * @param  ctrl      控制节点 (用于读取 mass_kg, com_length_m)
 * @param  position  当前关节角度 (rad)
 * @return 重力补偿力矩 (Nm)
 *
 * @note   公式: τ_g = m · g · L · sin(θ)
 *         假设: 关节旋转轴水平, θ=0 时连杆水平.
 *         四足机器人典型值: m=0.5~2.0kg, L=0.10~0.35m.
 */
float cg_calc_gravity_torque(const CyberGear_CtrlNode_t *ctrl, float position);

/**
 * @brief  计算摩擦补偿力矩
 * @param  ctrl      控制节点 (用于读取 coulomb_friction, viscous_friction)
 * @param  velocity  当前关节速度 (rad/s)
 * @return 摩擦补偿力矩 (Nm)
 *
 * @note   公式: τ_f = τ_coulomb · sign(ω) + b_viscous · ω
 *         库伦摩擦: 与速度方向相关, 与速度大小无关.
 *         粘滞摩擦: 与速度成正比.
 */
float cg_calc_friction_torque(const CyberGear_CtrlNode_t *ctrl, float velocity);

#ifdef __cplusplus
}
#endif

#endif /* __CYBERGEAR_CONTROL_H__ */
