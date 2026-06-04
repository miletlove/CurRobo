/**
 * @file    motor_service.c
 * @brief   电机生命周期管理服务实现
 * @author  CurRobo
 * @date    2026-06-05
 */
#include "motor_service.h"
#include "cybergear_motor.h"
#include "data_update.h"
#include "bsp_usart.h"

/* ================================================================
 *  外部引用 (main.c 定义)
 * ================================================================ */
extern CyberGear_CtrlNode_t g_cg_ctrl[];
#define MOTOR_COUNT 4   /* 与 wheel.h WHEEL_MOTOR_COUNT 一致 */

/* ================================================================
 *  内部状态
 * ================================================================ */
static MotorServiceNode_t g_motor_srv[MOTOR_COUNT];

/* ================================================================
 *  motor_service_init — 绑定控制节点, 初始化状态
 * ================================================================ */
void motor_service_init(void)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        g_motor_srv[i].ctrl            = &g_cg_ctrl[i];
        g_motor_srv[i].state           = MOTOR_STATE_OFFLINE;
        g_motor_srv[i].last_enable_ms  = 0;
        g_motor_srv[i].last_online_ms  = 0;
        g_motor_srv[i].enable_retries  = 0;
        g_motor_srv[i].fault_code      = 0;
    }
}

/* ================================================================
 *  motor_service_enable_all — 首次使能全部电机
 * ================================================================ */
void motor_service_enable_all(void)
{
    uint32_t now = data_update_get_tick_ms();

    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        CyberGear_CtrlNode_t *ctrl = g_motor_srv[i].ctrl;
        if (!ctrl || ctrl->enabled) continue;

        /* 速度模式 + 零速目标 */
        cg_ctrl_set_velocity(ctrl, 0.6f);  /* WHEEL_KD_VELOCITY */
        cg_ctrl_set_target(ctrl, 0.0f, 0.0f, 0.0f);

        /* 发送使能帧 */
        if (cg_ctrl_enable(ctrl) == 0)
        {
            g_motor_srv[i].state          = MOTOR_STATE_ENABLING;
            g_motor_srv[i].last_enable_ms = now;
            g_motor_srv[i].enable_retries = 1;
        }
    }
}

/* ================================================================
 *  motor_service_update — 周期检查 + 自动重试
 *
 *  逻辑:
 *    ① 同步 online 标志 (cg_ctrl_sync_online 已在 TIM6 ISR 中完成)
 *    ② 检查每个电机的状态
 *    ③ MOTOR_ENABLING: 检查是否已在线 → 转入 ONLINE
 *    ④ MOTOR_ONLINE:   检查是否超时离线 → 转入重试
 *    ⑤ MOTOR_OFFLINE:  如果 enable_retries > 0, 判断是否到重试间隔
 *    ⑥ MOTOR_FAULT:    等待故障清除
 *
 *  函数参数:
 *    无 (通过全局变量 g_motor_srv[] 访问)
 *
 *  函数输出:
 *    1=全部电机在线, 0=存在离线/故障
 * ================================================================ */
uint8_t motor_service_update(void)
{
    uint32_t now = data_update_get_tick_ms();
    uint8_t all_online = 1;

    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        CyberGear_CtrlNode_t *ctrl = g_motor_srv[i].ctrl;
        if (!ctrl) { all_online = 0; continue; }

        switch (g_motor_srv[i].state)
        {
        case MOTOR_STATE_ENABLING:
            /* 等待电机首次反馈 */
            if (ctrl->online)
            {
                g_motor_srv[i].state          = MOTOR_STATE_ONLINE;
                g_motor_srv[i].last_online_ms = now;
                g_motor_srv[i].fault_code     = 0;
                usart1_print("[MOTOR] ID=%u online (retry %u)\r\n",
                             ctrl->motor ? ctrl->motor->motor_id : 0,
                             g_motor_srv[i].enable_retries);
            }
            else if (now - g_motor_srv[i].last_enable_ms > MOTOR_ONLINE_TIMEOUT_MS)
            {
                /* 使能后未收到反馈: 可能电机未上电或 CAN 不通 */
                g_motor_srv[i].state = MOTOR_STATE_OFFLINE;
                usart1_print("[MOTOR] ID=%u enable timeout (no feedback)\r\n",
                             ctrl->motor ? ctrl->motor->motor_id : 0);
            }
            else
            {
                all_online = 0;
            }
            break;

        case MOTOR_STATE_ONLINE:
            /* 检测是否掉线 */
            if (ctrl->online)
            {
                g_motor_srv[i].last_online_ms = now;
                /* 检查故障码 */
                if (ctrl->motor && ctrl->motor->feedback.fault != 0)
                {
                    g_motor_srv[i].fault_code = ctrl->motor->feedback.fault;
                    g_motor_srv[i].state      = MOTOR_STATE_FAULT;
                    usart1_print("[MOTOR] ID=%u FAULT code=0x%02X\r\n",
                                 ctrl->motor->motor_id,
                                 g_motor_srv[i].fault_code);
                    all_online = 0;
                }
            }
            else if (now - g_motor_srv[i].last_online_ms > MOTOR_ONLINE_TIMEOUT_MS)
            {
                /* 超时未收到反馈 → 离线 */
                g_motor_srv[i].state = MOTOR_STATE_OFFLINE;
                usart1_print("[MOTOR] ID=%u offline (timeout)\r\n",
                             ctrl->motor ? ctrl->motor->motor_id : 0);
                all_online = 0;
            }
            break;

        case MOTOR_STATE_OFFLINE:
            /* 尝试重新使能 */
            if (g_motor_srv[i].enable_retries < MOTOR_MAX_RETRIES &&
                now - g_motor_srv[i].last_enable_ms > MOTOR_RETRY_INTERVAL_MS)
            {
                cg_ctrl_set_velocity(ctrl, 0.6f);
                cg_ctrl_set_target(ctrl, 0.0f, 0.0f, 0.0f);
                if (cg_ctrl_enable(ctrl) == 0)
                {
                    g_motor_srv[i].state          = MOTOR_STATE_ENABLING;
                    g_motor_srv[i].last_enable_ms = now;
                    g_motor_srv[i].enable_retries++;
                    usart1_print("[MOTOR] ID=%u retry enable (%u/%u)\r\n",
                                 ctrl->motor ? ctrl->motor->motor_id : 0,
                                 g_motor_srv[i].enable_retries,
                                 MOTOR_MAX_RETRIES);
                }
            }
            else if (g_motor_srv[i].enable_retries >= MOTOR_MAX_RETRIES)
            {
                /* 超过最大重试次数 → 保持离线, 等待外部干预 */
            }
            all_online = 0;
            break;

        case MOTOR_STATE_FAULT:
            /* 故障状态: 检查故障是否清除 */
            if (ctrl->motor && ctrl->motor->feedback.fault == 0)
            {
                g_motor_srv[i].fault_code = 0;
                g_motor_srv[i].state      = MOTOR_STATE_ONLINE;
                usart1_print("[MOTOR] ID=%u fault cleared\r\n",
                             ctrl->motor->motor_id);
            }
            all_online = 0;
            break;

        default:
            all_online = 0;
            break;
        }
    }

    return all_online;
}

/* ================================================================
 *  motor_service_get_state
 * ================================================================ */
MotorLifeState_t motor_service_get_state(uint8_t index)
{
    if (index >= MOTOR_COUNT) return MOTOR_STATE_OFFLINE;
    return g_motor_srv[index].state;
}

/* ================================================================
 *  motor_service_all_online
 * ================================================================ */
uint8_t motor_service_all_online(void)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        if (g_motor_srv[i].state != MOTOR_STATE_ONLINE)
            return 0;
    }
    return 1;
}

/* ================================================================
 *  motor_service_emergency_stop — 紧急停止 (发送 STOP 帧)
 * ================================================================ */
void motor_service_emergency_stop(void)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        CyberGear_CtrlNode_t *ctrl = g_motor_srv[i].ctrl;
        if (!ctrl) continue;
        cg_ctrl_stop(ctrl);
        g_motor_srv[i].state = MOTOR_STATE_OFFLINE;
    }
}

/* ================================================================
 *  motor_service_soft_stop — 软停止 (速度置零, 不发 STOP 帧)
 * ================================================================ */
void motor_service_soft_stop(void)
{
    for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        CyberGear_CtrlNode_t *ctrl = g_motor_srv[i].ctrl;
        if (!ctrl) continue;
        cg_ctrl_set_target(ctrl, 0.0f, 0.0f, 0.0f);
    }
}
