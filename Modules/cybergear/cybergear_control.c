/**
 * @file    cybergear_control.c
 * @brief   CyberGear 高级运控接口实现
 * @author  CurRobo
 * @date    2026-06-01
 *
 * @note    所有高级控制模式均基于 MIT 模式 (cg_motor_mit_control) 实现.
 *          调用链路:
 *            应用层状态机
 *              → cg_ctrl_set_mode()     设定模式 + 参数
 *              → cg_ctrl_set_target()   设定目标值
 *              → cg_ctrl_update()       每个控制周期调用 (1kHz)
 *                → 内部计算控制量
 *                → cg_motor_mit_control() 下发 CAN 帧
 *
 *          重力/摩擦补偿在 cg_ctrl_update() 内部自动叠加.
 *
 *          重要: 所有函数均为非阻塞 (除轨迹插值计算), 适合在 RTOS 任务或
 *               定时器中断中调用. 禁止在 CAN 接收中断中调用 cg_ctrl_update().
 */
#include "cybergear_control.h"
#include "data_update.h"  /* data_update_get_tick_ms(), 替代 HAL_GetTick */
#include <math.h>          /* sinf, fabsf, copysignf */
#include <string.h>        /* memset */

/* ================================================================
 *  物理常量
 * ================================================================ */
#define GRAVITY_ACCEL   (9.81f)       /* 重力加速度 (m/s²)           */
#define SIGN(x)         (((x) > 0.0f) ? 1.0f : (((x) < 0.0f) ? -1.0f : 0.0f))

/* ================================================================
 *  默认控制参数
 * ================================================================ */
#define DEFAULT_IMPEDANCE_K   (30.0f)    /* 默认刚度: 中等柔顺       */
#define DEFAULT_IMPEDANCE_D   (2.0f)     /* 默认阻尼: 适度           */
#define DEFAULT_PID_KP        (50.0f)    /* 默认位置比例增益          */
#define DEFAULT_PID_KI        (0.0f)     /* 默认无积分 (纯 PD)       */
#define DEFAULT_PID_KD        (1.0f)     /* 默认位置微分增益          */
#define DEFAULT_VEL_KD        (3.0f)     /* 默认速度阻尼增益          */
#define DEFAULT_MAX_TORQUE    (8.0f)     /* 默认力矩限幅 (Nm)        */
#define DEFAULT_INTEGRAL_LIMIT (2.0f)    /* 默认积分限幅 (Nm)        */

/* ================================================================
 *  内部辅助: 限幅函数
 * ================================================================ */
static inline float clampf(float x, float lo, float hi)
{
    if (x > hi) return hi;
    if (x < lo) return lo;
    return x;
}

/* ================================================================
 *  控制节点初始化
 *
 *  绑定电机对象, 设置默认控制参数, 模式初始为 IDLE.
 *
 *  函数参数:
 *    ctrl  : 控制节点指针 (应用层分配, 如全局数组)
 *    motor : 已调用 cg_motor_init() 初始化的电机对象指针
 *
 *  函数输出:
 *    ctrl 的所有字段被初始化为安全默认值.
 * ================================================================ */
void cg_ctrl_init(CyberGear_CtrlNode_t *ctrl, CyberGear_Motor_t *motor)
{
    if (ctrl == NULL || motor == NULL) return;

    memset(ctrl, 0, sizeof(CyberGear_CtrlNode_t));

    /* 绑定电机 */
    ctrl->motor = motor;

    /* 默认控制模式: 空闲 */
    ctrl->mode = CG_CTRL_MODE_IDLE;

    /* 默认阻抗参数 */
    ctrl->impedance.stiffness = DEFAULT_IMPEDANCE_K;
    ctrl->impedance.damping   = DEFAULT_IMPEDANCE_D;
    ctrl->impedance.inertia   = 0.0f;

    /* 默认 PID 参数 */
    ctrl->pid.kp             = DEFAULT_PID_KP;
    ctrl->pid.ki             = DEFAULT_PID_KI;
    ctrl->pid.kd             = DEFAULT_PID_KD;
    ctrl->pid.integral_limit = DEFAULT_INTEGRAL_LIMIT;
    ctrl->pid.max_torque     = DEFAULT_MAX_TORQUE;

    /* 补偿参数默认为 0 (不启用) */
    ctrl->mass_kg         = 0.0f;
    ctrl->com_length_m    = 0.0f;
    ctrl->coulomb_friction = 0.0f;
    ctrl->viscous_friction = 0.0f;

    /* 轨迹状态 */
    ctrl->traj.active = 0;

    /* 其他 */
    ctrl->enabled   = 0;
    ctrl->online    = 0;
    ctrl->last_update_tick = 0;
}

/* ================================================================
 *  模式切换函数
 * ================================================================ */

/**
 * @brief  切换到阻抗控制模式
 *
 *  阻抗控制律:
 *      τ = K · (θ_des - θ) + D · (ω_des - ω) + τ_ff
 *
 *  函数参数:
 *    ctrl      : 控制节点
 *    stiffness : 刚度 K (Nm/rad), 典型值 10~200
 *                - 值越大, 关节越"硬", 对外力抵抗越强
 *                - 值越小, 关节越"软", 越顺应外力
 *    damping   : 阻尼 D (Nm/(rad/s)), 典型值 0.5~10
 *                - 值越大, 振荡衰减越快, 但能耗增加
 *                - 值太小会导致振荡/抖动
 *
 *  理论推导 (阻抗控制):
 *    ─────────────────────────────────────────────────────────
 *    阻抗控制的核心思想是让电机表现出"质量-弹簧-阻尼"的二阶动力学特性:
 *
 *        M·(d²e/dt²) + D·(de/dt) + K·e = τ_ext
 *
 *    其中 e = θ_des - θ 为位置误差, τ_ext 为外部力矩.
 *
 *    对于 CyberGear MIT 模式, 电机内部已经实现了 PD 控制:
 *        τ_motor = KP_MIT · (θ_des - θ) + KD_MIT · (ω_des - ω) + τ_ff
 *
 *    因此我们将:
 *        KP_MIT = K (刚度)
 *        KD_MIT = D (阻尼)
 *        τ_ff   = 用户指定的前馈力矩 + 重力补偿 + 摩擦补偿
 *
 *    这样电机对外就表现出刚度为 K、阻尼为 D 的阻抗特性.
 *
 *    典型应用:
 *    - 四足机器人摆动相: K=10~50, D=0.5~3  (柔顺, 允许被环境推动)
 *    - 四足机器人站立相: K=100~300, D=3~10 (刚硬, 精确支撑体重)
 *    ─────────────────────────────────────────────────────────
 */
void cg_ctrl_set_impedance(CyberGear_CtrlNode_t *ctrl,
                           float stiffness, float damping)
{
    if (ctrl == NULL) return;

    ctrl->mode                 = CG_CTRL_MODE_IMPEDANCE;
    ctrl->impedance.stiffness  = clampf(stiffness, 0.0f, CG_KP_MAX);
    ctrl->impedance.damping    = clampf(damping,   0.0f, CG_KD_MAX);
    ctrl->integral_error       = 0.0f;  /* 模式切换时清零积分 */
}

/**
 * @brief  切换到位置 PID 控制模式
 *
 *  PID 控制律:
 *      e  = θ_des - θ
 *      τ  = Kp·e + Ki·∫e·dt + Kd·(de/dt) + τ_ff
 *
 *  函数参数:
 *    ctrl : 控制节点
 *    kp   : 比例增益 (Nm/rad)
 *           - Kp 决定了"弹簧"的刚度, 误差越大回复力矩越大
 *    ki   : 积分增益 (Nm/(rad·s))
 *           - Ki 用于消除稳态误差, 但会引起超调和积分饱和
 *           - 传 0 则退化为纯 PD 控制 (推荐大多数场景)
 *    kd   : 微分增益 (Nm/(rad/s))
 *           - Kd 提供阻尼, 抑制振荡, 提高稳定性
 *
 *  理论推导 (PID 位置控制):
 *    ─────────────────────────────────────────────────────────
 *    PID 控制器的传递函数:
 *        G_c(s) = Kp + Ki/s + Kd·s
 *
 *    离散化实现 (后向欧拉):
 *        e(k)     = θ_des(k) - θ(k)
 *        e_int(k) = e_int(k-1) + e(k) · Δt    (积分, 梯形法)
 *        e_dot(k) = (e(k) - e(k-1)) / Δt      (微分, 后向差分)
 *
 *        τ(k) = Kp·e(k) + Ki·e_int(k) + Kd·e_dot(k) + τ_ff
 *
 *    积分限幅 (anti-windup):
 *        if |e_int| > integral_limit:
 *            e_int = sign(e_int) · integral_limit
 *
 *    最终下发:
 *        MIT.KP = Kp  (利用电机内部位置环)
 *        MIT.KD = Kd  (利用电机内部速度环)
 *        τ_ff   = Ki·e_int + 重力补偿 + 摩擦补偿 + 用户 τ_ff
 *
 *    ⚠️ 注意: 积分项通过前馈力矩叠加, 因为 MIT 模式无积分通道.
 *    ─────────────────────────────────────────────────────────
 */
void cg_ctrl_set_position_pid(CyberGear_CtrlNode_t *ctrl,
                              float kp, float ki, float kd)
{
    if (ctrl == NULL) return;

    ctrl->mode          = CG_CTRL_MODE_POSITION;
    ctrl->pid.kp        = clampf(kp, 0.0f, CG_KP_MAX);
    ctrl->pid.ki        = clampf(ki, 0.0f, 100.0f); /* Ki 上限 100 */
    ctrl->pid.kd        = clampf(kd, 0.0f, CG_KD_MAX);
    ctrl->integral_error = 0.0f;  /* 模式切换时清零积分 */
}

/**
 * @brief  切换到速度控制模式
 *
 *  速度控制律:
 *      τ = Kd · (ω_des - ω) + τ_ff
 *
 *  函数参数:
 *    ctrl : 控制节点
 *    kd   : 速度阻尼增益 (Nm/(rad/s))
 *           - 值越大, 对速度误差的响应越强, 跟踪越快
 *           - 但过大可能导致振荡
 *
 *  实现方式:
 *    设置 MIT.KP = 0 (无位置偏差响应),
 *    MIT.KD = kd (通过速度环跟踪目标速度).
 *    位置目标设为当前值, 避免位置偏差产生额外力矩.
 *
 *  典型应用:
 *    - 电机转轮模式 (机器人转向)
 *    - 恒速旋转测试
 */
void cg_ctrl_set_velocity(CyberGear_CtrlNode_t *ctrl, float kd)
{
    if (ctrl == NULL) return;

    ctrl->mode       = CG_CTRL_MODE_VELOCITY;
    ctrl->pid.kd     = clampf(kd, 0.0f, CG_KD_MAX);
    ctrl->pid.kp     = 0.0f;   /* 纯速度模式: 关闭位置环 */
    ctrl->pid.ki     = 0.0f;
    ctrl->integral_error = 0.0f;
}

/**
 * @brief  切换到力矩 (开环) 控制模式
 *
 *  力矩控制律:
 *      τ = τ_des + τ_compensation
 *
 *  实现方式:
 *    设置 MIT.KP = 0, MIT.KD = 0 (关闭位置和速度环),
 *    MIT 的 torque 字段 = τ_des + 补偿力矩.
 *
 *  典型应用:
 *    - 零力矩模式 (让关节自由摆动)
 *    - 恒力测试 (施加恒定力矩)
 *    - 力控站立相 (根据足端力传感器反馈调节)
 */
void cg_ctrl_set_torque(CyberGear_CtrlNode_t *ctrl)
{
    if (ctrl == NULL) return;

    ctrl->mode = CG_CTRL_MODE_TORQUE;
    ctrl->pid.kp = 0.0f;
    ctrl->pid.kd = 0.0f;
    ctrl->pid.ki = 0.0f;
    ctrl->integral_error = 0.0f;
}

/* ================================================================
 *  目标值设定
 *
 *  不同模式使用的目标字段:
 *    IMPEDANCE  : position, velocity, torque_ff
 *    POSITION   : position, torque_ff
 *    VELOCITY   : velocity, torque_ff
 *    TORQUE     : torque_ff
 *    TRAJECTORY : (由轨迹点定义, 此函数不适用)
 * ================================================================ */
void cg_ctrl_set_target(CyberGear_CtrlNode_t *ctrl,
                        float position, float velocity, float torque_ff)
{
    if (ctrl == NULL) return;

    ctrl->target_position = clampf(position, CG_P_MIN, CG_P_MAX);
    ctrl->target_velocity = clampf(velocity, CG_V_MIN, CG_V_MAX);
    ctrl->target_torque   = clampf(torque_ff, CG_T_MIN, CG_T_MAX);
}

/* ================================================================
 *  轨迹接口
 *
 *  轨迹跟踪使用五次多项式插值 (最小 Jerk 轨迹):
 *
 *      定义归一化时间 s = t / T ∈ [0, 1], 其中 T = duration_ms.
 *
 *      位置:  θ(s) = θ_0 + (θ_f - θ_0) · p(s)
 *      速度:  ω(s) = (θ_f - θ_0) · v(s) / T
 *
 *      其中:
 *        p(s) = 10s³ - 15s⁴ + 6s⁵       (位移形状函数)
 *        v(s) = 30s² - 60s³ + 30s⁴       (速度形状函数, p 的导数)
 *
 *  理论推导 (最小 Jerk 轨迹):
 *    ─────────────────────────────────────────────────────────
 *    Jerk (加加速度) 是加速度的导数: j = d³θ/dt³.
 *
 *    最小 Jerk 轨迹最小化代价函数:
 *        J = ∫₀ᵀ (d³θ/dt³)² dt  → min
 *
 *    边界条件 (静止起止):
 *        θ(0)=θ₀, ω(0)=0, α(0)=0
 *        θ(T)=θ_f, ω(T)=0, α(T)=0
 *
 *    6 个边界条件 → 五次多项式:
 *        θ(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴ + a₅t⁵
 *
 *    代入边界条件解得:
 *        a₀ = θ₀, a₁ = 0, a₂ = 0
 *        a₃ = 10(θ_f - θ₀)/T³
 *        a₄ = -15(θ_f - θ₀)/T⁴
 *        a₅ = 6(θ_f - θ₀)/T⁵
 *
 *    归一化后得: θ(s) = θ₀ + (θ_f - θ₀)·(10s³ - 15s⁴ + 6s⁵)
 *
 *    优点:
 *    - 起止加速度为零 (无冲击)
 *    - 运动平滑, 适合机器人关节轨迹
 *    - 计算量小 (仅多项式求值)
 *    ─────────────────────────────────────────────────────────
 * ================================================================ */

/**
 * @brief  启动轨迹跟踪
 * @param  ctrl    控制节点
 * @param  points  轨迹点数组
 * @param  count   轨迹点数量 (至少 1 个)
 *
 *  函数功能:
 *    将 mode 切换为 TRAJECTORY, 初始化轨迹状态机.
 *    第一个轨迹点立即生效 (duration_ms=0 表示瞬时到达),
 *    后续点在 cg_ctrl_update() 中按时间推进.
 *
 *  典型用法:
 *    CyberGear_TrajPoint_t traj[] = {
 *        {.position=0.0f, .velocity=0.0f, .torque_ff=0.0f, .duration_ms=0},
 *        {.position=1.57f,.velocity=0.0f, .torque_ff=0.0f, .duration_ms=500},
 *        {.position=0.0f, .velocity=0.0f, .torque_ff=0.0f, .duration_ms=500},
 *    };
 *    cg_ctrl_start_trajectory(&ctrl, traj, 3);
 *    // 然后每个控制周期调用 cg_ctrl_update(&ctrl)
 */
void cg_ctrl_start_trajectory(CyberGear_CtrlNode_t *ctrl,
                              const CyberGear_TrajPoint_t *points,
                              uint8_t count)
{
    if (ctrl == NULL || points == NULL || count == 0) return;

    ctrl->mode = CG_CTRL_MODE_TRAJECTORY;

    /* 起始点 = 当前位置 (从反馈中读取) */
    if (ctrl->motor != NULL)
    {
        ctrl->traj.start.position  = ctrl->motor->feedback.position;
        ctrl->traj.start.velocity  = ctrl->motor->feedback.velocity;
        ctrl->traj.start.torque_ff = 0.0f;
    }
    else
    {
        memset(&ctrl->traj.start, 0, sizeof(CyberGear_TrajPoint_t));
    }
    ctrl->traj.start.duration_ms = 0;

    /* 第一个目标点 */
    ctrl->traj.target     = points[0];
    ctrl->traj.elapsed_ms = 0;
    ctrl->traj.active     = 1;

    /* 将剩余轨迹点存储 (简化: 仅存储 count, 由外部管理轨迹队列) */
    /* 注: 本实现仅支持当前目标点, 多段轨迹由应用层管理.
     *     如需自动推进到下一个点, 请在 cg_ctrl_update() 中检测
     *     elapsed_ms >= duration_ms 后, 由外部载入下一个点.
     */
    (void)count;   /* count 保留用于未来扩展 */
}

/* ================================================================
 *  重力补偿计算
 *
 *  公式:
 *      τ_g = m · g · L · sin(θ)
 *
 *  函数参数:
 *    ctrl     : 控制节点 (读取 mass_kg, com_length_m)
 *    position : 当前关节角度 (rad)
 *
 *  返回值:
 *    重力补偿力矩 (Nm), 若 mass_kg=0 或 com_length_m=0 则返回 0.
 *
 *  理论推导 (单关节重力补偿):
 *    ─────────────────────────────────────────────────────────
 *    考虑一个绕水平轴旋转的连杆:
 *
 *        - 质心到转轴距离: L (m)
 *        - 连杆质量:       m (kg)
 *        - 连杆与水平面夹角: θ (rad)
 *
 *    重力对转轴的力矩:
 *        τ_g = m · g · L · cos(θ)    (θ 从竖直向上测量)
 *    或:
 *        τ_g = m · g · L · sin(θ)    (θ 从水平方向测量)
 *
 *    本实现假设 θ=0 时连杆水平 (力矩臂最大),
 *    因此使用 sin(θ) 形式.
 *
 *    对于四足机器人:
 *    - 髋关节: 需同时考虑大腿 + 小腿的质量和质心
 *    - 膝关节: 仅需考虑小腿质量
 *    - 实际使用时可用等效参数 (m_eff, L_eff) 简化
 *
 *    多连杆串联时, 使用递推 Newton-Euler 法可精确计算,
 *    但计算量大, 本函数提供单关节近似.
 *    ─────────────────────────────────────────────────────────
 * ================================================================ */
float cg_calc_gravity_torque(const CyberGear_CtrlNode_t *ctrl, float position)
{
    if (ctrl == NULL) return 0.0f;
    if (ctrl->mass_kg <= 0.0f || ctrl->com_length_m <= 0.0f) return 0.0f;

    return ctrl->mass_kg * GRAVITY_ACCEL * ctrl->com_length_m * sinf(position);
}

/**
 * @brief  设置重力补偿参数
 */
void cg_ctrl_set_gravity_comp(CyberGear_CtrlNode_t *ctrl,
                              float mass_kg, float com_length_m)
{
    if (ctrl == NULL) return;
    ctrl->mass_kg      = (mass_kg > 0.0f)      ? mass_kg      : 0.0f;
    ctrl->com_length_m = (com_length_m > 0.0f) ? com_length_m : 0.0f;
}

/* ================================================================
 *  摩擦补偿计算
 *
 *  公式:
 *      τ_f = τ_coulomb · sign(ω) + b_viscous · ω
 *
 *  函数参数:
 *    ctrl     : 控制节点 (读取 coulomb_friction, viscous_friction)
 *    velocity : 当前关节速度 (rad/s)
 *
 *  返回值:
 *    摩擦补偿力矩 (Nm)
 *
 *  理论推导 (摩擦模型):
 *    ─────────────────────────────────────────────────────────
 *    关节摩擦通常包含三个分量:
 *
 *    1. 库伦摩擦 (Coulomb Friction) τ_c:
 *       - 与速度方向有关, 与速度大小无关
 *       - 物理原因: 接触面的静摩擦
 *       - 模型: τ_c · sign(ω)
 *       - 典型值: 0.05~0.3 Nm (小型电机)
 *
 *    2. 粘滞摩擦 (Viscous Friction) b:
 *       - 与速度成正比
 *       - 物理原因: 润滑剂/空气阻力
 *       - 模型: b · ω
 *       - 典型值: 0.01~0.1 Nm/(rad/s)
 *
 *    3. 静摩擦 (Stiction) — 本实现暂忽略:
 *       - 仅在 ω=0 时存在
 *       - 需要额外的 breakaway 逻辑
 *
 *    总摩擦补偿力矩:
 *        τ_f = τ_c · sign(ω) + b · ω
 *
 *    注意事项:
 *    - sign(0) 的处理: 本实现返回 0 (避免在零速附近抖动)
 *    - 摩擦参数可通过系统辨识实验测定:
 *      匀速运动测 τ vs ω → 斜率为 b, 截距为 τ_c
 *    ─────────────────────────────────────────────────────────
 * ================================================================ */
float cg_calc_friction_torque(const CyberGear_CtrlNode_t *ctrl, float velocity)
{
    if (ctrl == NULL) return 0.0f;

    float tau_f = 0.0f;

    /* 库伦摩擦 */
    if (ctrl->coulomb_friction > 0.0f)
    {
        /* 设置死区: |ω| < 0.01 rad/s 时视为静止, 不施加库伦补偿 */
        if (fabsf(velocity) > 0.01f)
        {
            tau_f += copysignf(ctrl->coulomb_friction, velocity);
        }
    }

    /* 粘滞摩擦 */
    tau_f += ctrl->viscous_friction * velocity;

    return tau_f;
}

/**
 * @brief  设置摩擦补偿参数
 */
void cg_ctrl_set_friction_comp(CyberGear_CtrlNode_t *ctrl,
                               float coulomb, float viscous)
{
    if (ctrl == NULL) return;
    ctrl->coulomb_friction = (coulomb > 0.0f) ? coulomb : 0.0f;
    ctrl->viscous_friction = (viscous > 0.0f) ? viscous : 0.0f;
}

/* ================================================================
 *  核心更新函数 (固定 dt 版本)
 *
 *  每个控制周期调用一次.
 *
 *  执行流程:
 *    1. 检查电机是否在线且已使能
 *    2. 根据当前 mode 计算控制量 (position, velocity, kp, kd, torque_ff)
 *    3. 叠加重力补偿 + 摩擦补偿
 *    4. 通过 cg_motor_mit_control() 下发
 *
 *  函数参数:
 *    ctrl : 控制节点
 *    dt   : 控制周期 (秒), TIM6 1kHz 时固定传 0.001f
 *
 *  返回值:
 *    0 = 成功下发
 *    1 = 失败 (电机不在线 / 未使能 / 发送失败)
 *
 *  ⚠️ ISR 安全性:
 *    - HAL_FDCAN_AddMessageToTxFifoQ(): ISR 安全 ✓
 *    - 浮点运算: CM7 + FPU, 编译器自动保存 FPU 上下文 ✓
 *    - 禁止在 CAN Rx ISR (优先级高于 TIM6) 中调用 — 会导致重入
 * ================================================================ */
uint8_t cg_ctrl_update_fixed(CyberGear_CtrlNode_t *ctrl, float dt)
{
    if (ctrl == NULL || ctrl->motor == NULL) return 1;
    if (ctrl->mode == CG_CTRL_MODE_IDLE)    return 0;   /* 空闲模式, 不发送 */
    if (!ctrl->online || !ctrl->enabled)     return 1;   /* 不在线或未使能 */

    CyberGear_Motor_t   *motor = ctrl->motor;
    CyberGear_Feedback_t *fb   = &motor->feedback;

    /* ---- 1. dt 由调用方提供 (TIM6 → 0.001f) ---- */

    /* ---- 2. 根据模式计算基本控制量 ---- */
    float position_cmd = fb->position;   /* 默认: 保持当前位置 */
    float velocity_cmd = 0.0f;
    float torque_ff    = ctrl->target_torque;
    float kp_cmd       = 0.0f;
    float kd_cmd       = 0.0f;

    switch (ctrl->mode)
    {
    /* ============================================
     *  阻抗控制
     *
     *  控制律:
     *      τ = K·(θ_des - θ) + D·(ω_des - ω) + τ_ff
     *
     *  实现:
     *      KP_MIT = K (刚度)
     *      KD_MIT = D (阻尼)
     *      pos_cmd = θ_des, vel_cmd = ω_des
     * ============================================ */
    case CG_CTRL_MODE_IMPEDANCE:
        kp_cmd       = ctrl->impedance.stiffness;
        kd_cmd       = ctrl->impedance.damping;
        position_cmd = ctrl->target_position;
        velocity_cmd = ctrl->target_velocity;
        break;

    /* ============================================
     *  位置 PID 控制
     *
     *  控制律:
     *      e    = θ_des - θ
     *      e_int = ∫e·dt  (梯形积分 + 限幅)
     *      τ    = Kp·e + Ki·e_int + Kd·(de/dt) + τ_ff
     *
     *  实现:
     *      KP_MIT = Kp
     *      KD_MIT = Kd
     *      torque_ff += Ki * e_int  (积分项叠加到前馈)
     *      pos_cmd = θ_des, vel_cmd = 0
     * ============================================ */
    case CG_CTRL_MODE_POSITION:
    {
        float error = ctrl->target_position - fb->position;

        /* 积分 (梯形法, 带 anti-windup) */
        ctrl->integral_error += error * dt;

        /* 积分限幅 */
        float ilim = ctrl->pid.integral_limit;
        ctrl->integral_error = clampf(ctrl->integral_error, -ilim, +ilim);

        torque_ff += ctrl->pid.ki * ctrl->integral_error;

        kp_cmd       = ctrl->pid.kp;
        kd_cmd       = ctrl->pid.kd;
        position_cmd = ctrl->target_position;
        velocity_cmd = 0.0f;
        break;
    }

    /* ============================================
     *  速度控制
     *
     *  控制律:
     *      τ = Kd·(ω_des - ω) + τ_ff
     *
     *  实现:
     *      KP_MIT = 0 (关闭位置偏差响应)
     *      KD_MIT = Kd
     *      pos_cmd = 当前实际位置 (不给位置指令, 避免位置偏差)
     *      vel_cmd = ω_des
     * ============================================ */
    case CG_CTRL_MODE_VELOCITY:
        kp_cmd       = 0.0f;
        kd_cmd       = ctrl->pid.kd;
        position_cmd = fb->position;       /* 跟随当前位置 */
        velocity_cmd = ctrl->target_velocity;
        break;

    /* ============================================
     *  力矩控制 (开环)
     *
     *  控制律:
     *      τ = τ_des
     *
     *  实现:
     *      KP_MIT = 0, KD_MIT = 0
     *      torque_ff = τ_des + 补偿
     *      pos_cmd = 当前位置
     * ============================================ */
    case CG_CTRL_MODE_TORQUE:
        kp_cmd       = 0.0f;
        kd_cmd       = 0.0f;
        position_cmd = fb->position;       /* 无位置指令 */
        velocity_cmd = 0.0f;
        /* torque_ff 已在上面赋值 ctrl->target_torque */
        break;

    /* ============================================
     *  轨迹跟踪 (五次多项式插值)
     *
     *  控制律:
     *      s = t / T
     *      p(s) = 10s³ - 15s⁴ + 6s⁵
     *      v(s) = 30s² - 60s³ + 30s⁴
     *      θ_cmd(s) = θ_start + (θ_target - θ_start) · p(s)
     *      ω_cmd(s) = (θ_target - θ_start) · v(s) / T
     *
     *  实现:
     *      KP_MIT = 阻抗刚度 (默认 50), KD_MIT = 阻抗阻尼 (默认 2)
     *      pos_cmd = θ_cmd(s), vel_cmd = ω_cmd(s)
     * ============================================ */
    case CG_CTRL_MODE_TRAJECTORY:
    {
        if (!ctrl->traj.active)
        {
            /* 轨迹已完成, 保持最后位置 */
            kp_cmd       = ctrl->impedance.stiffness;
            kd_cmd       = ctrl->impedance.damping;
            position_cmd = ctrl->traj.target.position;
            velocity_cmd = 0.0f;
            break;
        }

        /* 推进时间 */
        ctrl->traj.elapsed_ms += (uint32_t)(dt * 1000.0f);

        uint32_t T_ms = ctrl->traj.target.duration_ms;
        if (T_ms == 0) T_ms = 1;  /* 防除零 */

        /* 归一化时间 s ∈ [0, 1] */
        float s = (float)ctrl->traj.elapsed_ms / (float)T_ms;
        if (s >= 1.0f)
        {
            /* 到达目标点 */
            s = 1.0f;
            ctrl->traj.active = 0;  /* 轨迹结束 */
        }

        /* 五次多项式形状函数 */
        float s2 = s * s;
        float s3 = s2 * s;
        float s4 = s3 * s;
        float s5 = s4 * s;

        float p_shape = 10.0f * s3 - 15.0f * s4 + 6.0f * s5;
        float v_shape = 30.0f * s2 - 60.0f * s3 + 30.0f * s4;

        float dp = ctrl->traj.target.position - ctrl->traj.start.position;

        /* 插值位置 (线性插值前馈力矩) */
        position_cmd = ctrl->traj.start.position + dp * p_shape;
        velocity_cmd = dp * v_shape / ((float)T_ms * 0.001f);

        /* 前馈力矩: 线性插值 + 用户设定的 torque_ff */
        torque_ff = ctrl->traj.start.torque_ff
                    + (ctrl->traj.target.torque_ff - ctrl->traj.start.torque_ff) * s;
        torque_ff += ctrl->target_torque;

        /* 使用阻抗参数作为跟踪增益 */
        kp_cmd = ctrl->impedance.stiffness;
        kd_cmd = ctrl->impedance.damping;

        break;
    }

    default:
        return 0;   /* 未知模式, 不下发 */
    }

    /* ---- 3. 叠加重力补偿 ---- */
    torque_ff += cg_calc_gravity_torque(ctrl, fb->position);

    /* ---- 4. 摩擦补偿 (暂时禁用, 后续通过参数辨识启用) ---- */
    /* torque_ff += cg_calc_friction_torque(ctrl, fb->velocity); */

    /* ---- 5. 力矩输出限幅 ---- */
    torque_ff = clampf(torque_ff, -ctrl->pid.max_torque, +ctrl->pid.max_torque);

    /* ---- 6. 构造 MIT 命令并下发 ---- */
    CyberGear_MITCmd_t mit_cmd;
    mit_cmd.position = clampf(position_cmd, CG_P_MIN, CG_P_MAX);
    mit_cmd.velocity = clampf(velocity_cmd, CG_V_MIN, CG_V_MAX);
    mit_cmd.torque   = torque_ff;
    mit_cmd.kp       = kp_cmd;
    mit_cmd.kd       = kd_cmd;

    return cg_motor_mit_control(motor, &mit_cmd);
}

/**
 * @brief  cg_ctrl_update() — 向后兼容包装, 自动计算 dt
 *
 *  适用场景: FreeRTOS 任务或主循环中调用 (非固定频率).
 *  固定频率 (如 TIM6 1kHz) 推荐直接使用 cg_ctrl_update_fixed().
 */
uint8_t cg_ctrl_update(CyberGear_CtrlNode_t *ctrl)
{
    if (ctrl == NULL || ctrl->motor == NULL) return 1;

    uint32_t now_tick = data_update_get_tick_ms();
    float    dt;
    if (ctrl->last_update_tick == 0)
    {
        dt = 0.001f;
    }
    else
    {
        uint32_t dt_ms = now_tick - ctrl->last_update_tick;
        if (dt_ms == 0) dt_ms = 1;
        dt = (float)dt_ms * 0.001f;
    }
    ctrl->last_update_tick = now_tick;

    return cg_ctrl_update_fixed(ctrl, dt);
}

/* ================================================================
 *  电机使能 / 停止 (带控制节点管理)
 * ================================================================ */

/**
 * @brief  电机使能
 *
 *  函数功能:
 *    1. 发送使能 CAN 帧
 *    2. 将 ctrl->enabled 置 1
 *    3. 清零积分误差 (避免使能瞬间的积分冲击)
 *
 *  调用时机:
 *    在确认电机在线后调用 (如收到反馈帧后).
 */
uint8_t cg_ctrl_enable(CyberGear_CtrlNode_t *ctrl)
{
    if (ctrl == NULL || ctrl->motor == NULL) return 1;

    uint8_t ret = cg_motor_enable(ctrl->motor);
    if (ret == 0)
    {
        ctrl->enabled        = 1;
        ctrl->integral_error = 0.0f;
        ctrl->last_update_tick = data_update_get_tick_ms();  /* TIM6 驱动 */
    }
    return ret;
}

/**
 * @brief  电机停止
 *
 *  函数功能:
 *    1. 发送停止 CAN 帧
 *    2. 将 ctrl->enabled 置 0
 *    3. 模式切换到 IDLE
 */
uint8_t cg_ctrl_stop(CyberGear_CtrlNode_t *ctrl)
{
    if (ctrl == NULL || ctrl->motor == NULL) return 1;

    uint8_t ret = cg_motor_stop(ctrl->motor);
    ctrl->enabled        = 0;
    ctrl->mode           = CG_CTRL_MODE_IDLE;
    ctrl->integral_error = 0.0f;
    ctrl->traj.active    = 0;
    return ret;
}
