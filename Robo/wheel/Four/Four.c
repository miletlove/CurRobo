/**
 * @file    Four.c
 * @brief   Four — 四驱差速运动模块实现 (普通橡胶轮, 无转向架)
 * @author  CurRobo
 * @date    2026-10-09
 *
 * @note    差速模型:
 *            v_left  = (vx - wz*GAIN) * LEFT_DIR_SIGN   → ID=1,3
 *            v_right = (vx + wz*GAIN) * RIGHT_DIR_SIGN  → ID=2,4
 *
 *          摇杆满偏左右 → 左列与右列等速反向 → 原地旋转.
 *          等比限幅: 任一轮超限时整体缩放, 保持运动方向.
 */
#include "Four.h"
#include "cybergear_control.h"
#include "remote_control.h"
#include "motor_service.h"
#include <math.h>

/* ================================================================
 *  外部引用 (main.c 定义, pipeline 绑定)
 * ================================================================ */
extern CyberGear_CtrlNode_t g_cg_ctrl[];

/* ================================================================
 *  内部辅助
 * ================================================================ */
static inline float four_clamp_sym(float x, float limit)
{
    if (x >  limit) return  limit;
    if (x < -limit) return -limit;
    return x;
}

/* ================================================================
 *  four_init — 速度模式 + 零目标 + 使能
 * ================================================================ */
void four_init(void)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        cg_ctrl_set_velocity(&g_cg_ctrl[i], FOUR_KD_VELOCITY);
        cg_ctrl_set_target(&g_cg_ctrl[i], 0.0f, 0.0f, 0.0f);
        cg_ctrl_enable(&g_cg_ctrl[i]);
    }
}

/* ================================================================
 *  four_sw_update — 使能开关处理 (任意 FSM 态生效)
 * ================================================================ */
void four_sw_update(void)
{
    const RC_ctrl_t *rc = get_remote_control_point();

    if (SW_IS_UP(rc->rc.s[FOUR_ENABLE_SW_INDEX]))
    {
        /* UP: 使能 (motor_service 自动使能/重试, 重试计数已重置) */
        motor_service_set_enable(1);
    }
    else
    {
        /* MID(3) / DOWN(2): 关闭, 不使能 */
        motor_service_set_enable(0);
    }
}

/* ================================================================
 *  four_update — 差速解算 (仅 FSM=READY 态调用)
 * ================================================================ */
void four_update(void)
{
    const RC_ctrl_t *rc = get_remote_control_point();

    /* ---- ① 右摇杆 → 直行 + 转向分量 ---- */
    /* ch[1] 上下: 前进(+)/后退(-) */
    float vx = (float)rc->rc.ch[1] / (float)FOUR_RC_CH_MAX_ABS
               * FOUR_MAX_SPEED_RAD_S;
    /* ch[0] 左右: 左转(+)/右转(-), 差速转向 */
    float wz = (float)rc->rc.ch[0] / (float)FOUR_RC_CH_MAX_ABS
               * FOUR_MAX_TURN_RAD_S;

    vx = four_clamp_sym(vx, FOUR_MAX_SPEED_RAD_S);
    wz = four_clamp_sym(wz, FOUR_MAX_TURN_RAD_S);

    /* ---- ③ 差速解算
     *  左列 (电机+ = 前进):  L = (vx + wz*GAIN) * (+1)
     *  右列 (电机+ = 后退):  R = (vx - wz*GAIN) * (-1)
     *  验证:
     *    全前 (vx=+2, wz=0): L=+2, R=-2 → 左右物理同向前进 ✓
     *    全右 (vx=0, wz=+2): L=+2, R=+2 → 左前右后 → 原地右转 ✓
     */
    float v_left  = (vx + wz * FOUR_TURN_GAIN) * FOUR_LEFT_DIR_SIGN;
    float v_right = (vx - wz * FOUR_TURN_GAIN) * FOUR_RIGHT_DIR_SIGN;

    /* ---- ④ 等比限幅: 单轮上限 = 拨杆满量程对应最大转速 ---- */
    float max_abs = fabsf(v_left);
    if (fabsf(v_right) > max_abs) max_abs = fabsf(v_right);

    if (max_abs > FOUR_MAX_SPEED_RAD_S)
    {
        float scale = FOUR_MAX_SPEED_RAD_S / max_abs;
        v_left  *= scale;
        v_right *= scale;
    }

    /* ---- ⑤ 写入目标 (CAN 帧由 TIM6 ISR 500Hz 发送) ---- */
    cg_ctrl_set_target(&g_cg_ctrl[FOUR_MOTOR_FL], 0.0f, v_left,  0.0f);
    cg_ctrl_set_target(&g_cg_ctrl[FOUR_MOTOR_FR], 0.0f, v_right, 0.0f);
    cg_ctrl_set_target(&g_cg_ctrl[FOUR_MOTOR_RL], 0.0f, v_left,  0.0f);
    cg_ctrl_set_target(&g_cg_ctrl[FOUR_MOTOR_RR], 0.0f, v_right, 0.0f);
}
