/**
 * @file    sys_monitor.c
 * @brief   系统联调观察模块实现 — 串口打印电机状态与遥控器数据
 * @author  CurRobo
 * @date    2026-10-09
 *
 * @note    时基: TIM6 1kHz (data_update_get_tick_ms).
 *          打印: usart1_print() (USART1 DMA TX, 非阻塞).
 *          只读: remote_ctrl / g_cg_ctrl / motor_service 状态,
 *                不影响控制链路.
 */
#include "sys_monitor.h"
#include "data_update.h"
#include "motor_service.h"
#include "remote_control.h"
#include "bsp_usart.h"
#include "fdcan.h"

/* ================================================================
 *  外部引用 (main.c 定义, pipeline 绑定)
 * ================================================================ */
extern CyberGear_CtrlNode_t g_cg_ctrl[];

/* ================================================================
 *  打印周期 (ms), TIM6 时基
 * ================================================================ */
#define MON_RC_PERIOD_MS      100     /* 遥控数据 10Hz */
#define MON_MOTOR_PERIOD_MS   1000    /* 电机状态 1Hz */

/* ================================================================
 *  生命周期状态名
 * ================================================================ */
static const char *mon_state_name(MotorLifeState_t s)
{
    switch (s)
    {
    case MOTOR_STATE_OFFLINE:  return "OFFLINE";
    case MOTOR_STATE_ENABLING: return "ENABLING";
    case MOTOR_STATE_ONLINE:   return "ONLINE";
    case MOTOR_STATE_FAULT:    return "FAULT";
    default:                   return "?";
    }
}

/* ================================================================
 *  sys_monitor_init
 * ================================================================ */
void sys_monitor_init(void)
{
    usart1_print("\r\n======== System Integration Monitor ========\r\n");
    usart1_print("[MON] motor state: 1Hz | RC data: 10Hz | timebase: TIM6\r\n");
}

/* ================================================================
 *  sys_monitor_run — 主循环每周期调用
 * ================================================================ */
void sys_monitor_run(void)
{
    static uint32_t last_rc    = 0;
    static uint32_t last_motor = 0;
    uint32_t now = data_update_get_tick_ms();

    /* ---- 遥控器数据: 10Hz ---- */
    if (now - last_rc >= MON_RC_PERIOD_MS)
    {
        last_rc = now;

        const RC_ctrl_t *rc = get_remote_control_point();
        if (rc->online)
        {
            usart1_print("[RC] ch:%5d %5d %5d %5d %5d | sw:%u/%u | key:0x%04X\r\n",
                         rc->rc.ch[0], rc->rc.ch[1], rc->rc.ch[2],
                         rc->rc.ch[3], rc->rc.ch[4],
                         rc->rc.s[0], rc->rc.s[1], rc->key.v);
        }
        else
        {
            usart1_print("[RC] offline\r\n");
        }
    }

    /* ---- 电机状态: 1Hz ---- */
    if (now - last_motor >= MON_MOTOR_PERIOD_MS)
    {
        last_motor = now;

        for (uint8_t i = 0; i < 4; i++)
        {
            CyberGear_CtrlNode_t *ctrl  = &g_cg_ctrl[i];
            CyberGear_Motor_t    *motor = ctrl->motor;
            if (!motor) continue;

            usart1_print("[MON] M%u id=%u %s state=%-8s online=%u en=%u "
                         "pos=%+8.3f vel=%+8.3f temp=%.1f fault=0x%02X\r\n",
                         i + 1,
                         motor->motor_id,
                         (motor->hcan == &hfdcan1) ? "CAN1" : "CAN2",
                         mon_state_name(motor_service_get_state(i)),
                         ctrl->online,
                         ctrl->enabled,
                         (double)motor->feedback.position,
                         (double)motor->feedback.velocity,
                         (double)motor->feedback.temperature,
                         motor->feedback.fault);
        }
    }
}
