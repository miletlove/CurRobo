/**
 * @file    wheel.c
 * @brief   四轮麦克纳姆轮运动控制模块实现 (X型布局)
 * @author  CurRobo
 * @date    2026-06-04
 *
 * @note    控制链路:
 *            main() while(1)
 *              → app_task_run()
 *                → app_task_wheel_update()    [安全检查: 遥控器/电机在线/使能]
 *                  → wheel_update()           [运动学解算 + 目标写入]
 *
 *           CAN 帧由 TIM6 ISR (500Hz) 中的 data_update_task_motor()
 *           → cg_ctrl_update_fixed() 自动发送, 本模块不直接操作 CAN.
 *
 *           X 型麦克纳姆轮逆运动学 (纯平移, 无旋转):
 *            ┌───────────────────────────────────────────┐
 *            │  v_fl = vx - vy    (前左, ID=1, 索引0)     │
 *            │  v_rl = vx + vy    (后左, ID=2, 索引1)     │
 *            │  v_fr = vx + vy    (前右, ID=3, 索引2)     │
 *            │  v_rr = vx - vy    (后右, ID=4, 索引3)     │
 *            │                                           │
 *            │  vx: 前进 (+) / 后退 (-)                   │
 *            │  vy: 左移 (+) / 右移 (-)                   │
 *            └───────────────────────────────────────────┘
 *
 *           速度比例缩放:
 *            当任一车轮合成速度超出 WHEEL_MAX_SPEED_RAD_S 时,
 *            等比缩放所有轮速以保持运动方向, 最大合成速度不超限幅.
 *
 *           例如: vx=2.0, vy=2.0 → v_rl=v_fr=4.0 (超限)
 *                 scale = 2.0/4.0 = 0.5
 *                 → v_rl_s=v_fr_s=2.0, v_fl_s=v_rr_s=0.0
 *                 实际运动: 45° 方向以 2.0 rad/s 移动
 */
#include "wheel.h"
#include "cybergear_control.h"
#include "remote_control.h"
#include <math.h>

/* ================================================================
 *  外部引用 (main.c 定义)
 * ================================================================ */
extern CyberGear_CtrlNode_t g_cg_ctrl[];

/* ================================================================
 *  内部辅助
 * ================================================================ */

/** 钳位值在 [-limit, +limit] 范围内 */
static inline float clamp_sym(float x, float limit)
{
    if (x >  limit) return  limit;
    if (x < -limit) return -limit;
    return x;
}

/* ================================================================
 *  wheel_init — 四轮初始化
 * ================================================================ */
void wheel_init(void)
{
    for (uint8_t i = 0; i < WHEEL_MOTOR_COUNT; i++)
    {
        /* 速度模式: Kp=0 (不跟踪位置), Kd=WHEEL_KD_VELOCITY (跟踪速度) */
        cg_ctrl_set_velocity(&g_cg_ctrl[i], WHEEL_KD_VELOCITY);

        /* 初始目标速度 = 0 */
        cg_ctrl_set_target(&g_cg_ctrl[i], 0.0f, 0.0f, 0.0f);

        /* 发送使能帧 */
        cg_ctrl_enable(&g_cg_ctrl[i]);
    }
}

/* ================================================================
 *  wheel_update — 摇杆 → 四轮速度映射
 * ================================================================ */
void wheel_update(void)
{
    /* 1. 读取遥控器通道
     *    ch[0]: 前进 (+)/后退 (-), 对应 vx
     *    ch[1]: 左移 (+)/右移 (-), 对应 vy
     *    注意: SBUS 协议中通道值已减去 1024 中位偏移
     */
    int16_t ch0 = remote_ctrl.rc.ch[0];
    int16_t ch1 = remote_ctrl.rc.ch[1];

    /* 2. 线性映射: RC 原始值 → 车身目标速度 (rad/s)
     *    vx = (ch0 / 660) × 2.0
     *    vy = (ch1 / 660) × 2.0
     */
    float vx = (float)ch0 / (float)WHEEL_RC_CH_MAX_ABS * WHEEL_MAX_SPEED_RAD_S;
    float vy = (float)ch1 / (float)WHEEL_RC_CH_MAX_ABS * WHEEL_MAX_SPEED_RAD_S;

    /* 钳位: 单个摇杆方向不超限 */
    vx = clamp_sym(vx, WHEEL_MAX_SPEED_RAD_S);
    vy = clamp_sym(vy, WHEEL_MAX_SPEED_RAD_S);

    /* 3. X 型麦克纳姆轮逆运动学: 车身速度 → 各轮速度 */
    float v_fl =  vx - vy;    /* 前左 (ID=1, 索引0) */
    float v_rl =  vx + vy;    /* 后左 (ID=2, 索引1) */
    float v_fr =  vx + vy;    /* 前右 (ID=3, 索引2) */
    float v_rr =  vx - vy;    /* 后右 (ID=4, 索引3) */

    /* 4. 等比缩放: 当任一车轮速度超出限幅时, 整体等比缩放
     *    保证运动方向不变, 仅等比例降低速度
     */
    float max_abs = fabsf(v_fl);
    if (fabsf(v_rl) > max_abs) max_abs = fabsf(v_rl);
    if (fabsf(v_fr) > max_abs) max_abs = fabsf(v_fr);
    if (fabsf(v_rr) > max_abs) max_abs = fabsf(v_rr);

    if (max_abs > WHEEL_MAX_SPEED_RAD_S)
    {
        float scale = WHEEL_MAX_SPEED_RAD_S / max_abs;
        v_fl *= scale;
        v_rl *= scale;
        v_fr *= scale;
        v_rr *= scale;
    }

    /* 5. 更新各电机控制目标 (仅写内存, CAN 帧由 TIM6 ISR 发送) */
    cg_ctrl_set_target(&g_cg_ctrl[WHEEL_MOTOR_FL], 0.0f, v_fl, 0.0f);
    cg_ctrl_set_target(&g_cg_ctrl[WHEEL_MOTOR_RL], 0.0f, v_rl, 0.0f);
    cg_ctrl_set_target(&g_cg_ctrl[WHEEL_MOTOR_FR], 0.0f, v_fr, 0.0f);
    cg_ctrl_set_target(&g_cg_ctrl[WHEEL_MOTOR_RR], 0.0f, v_rr, 0.0f);
}
