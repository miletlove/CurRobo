/**
 * @file    remote_control.c
 * @brief   遥控器数据管理 — SBUS 解析 + 全局数据
 * @note    remote_ctrl 由 bsp_rc.c 的 ISR 直接更新, 主循环只读
 *          最后接收时间戳在 ISR 中记录 (TIM6 时基),
 *          online 超时清零由 remote_control_update() 在主循环完成.
 */
#include "remote_control.h"
#include "bsp_rc.h"
#include "data_update.h"
#include <string.h>

/* 全局遥控器数据 (ISR 写入, 主循环读取) */
RC_ctrl_t remote_ctrl;

/* 最后一次成功接收帧的时间戳 (ISR 写入, 主循环读取) */
static volatile uint32_t s_rc_last_rx_ms = 0;

/* ================================================================
 *  sbus_to_rc — 18 字节 SBUS → RC_ctrl_t (在 ISR 中调用)
 * ================================================================ */
void sbus_to_rc(const uint8_t *buf, RC_ctrl_t *rc)
{
    if (!buf || !rc) return;

    rc->rc.ch[0] = (int16_t)(((buf[0]  | (buf[1]  << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET);
    rc->rc.ch[1] = (int16_t)((((buf[1] >> 3) | (buf[2]  << 5)) & 0x07FF) - RC_CH_VALUE_OFFSET);
    rc->rc.ch[2] = (int16_t)((((buf[2] >> 6) | (buf[3]  << 2) | (buf[4] << 10)) & 0x07FF) - RC_CH_VALUE_OFFSET);
    rc->rc.ch[3] = (int16_t)((((buf[4] >> 1) | (buf[5]  << 7)) & 0x07FF) - RC_CH_VALUE_OFFSET);
    rc->rc.ch[4] = (int16_t)(((buf[16] | (buf[17] << 8))) - RC_CH_VALUE_OFFSET);

    rc->rc.s[0] = (RC_Switch_t)((buf[5] >> 4) & 0x0003);
    rc->rc.s[1] = (RC_Switch_t)(((buf[5] >> 4) & 0x000C) >> 2);

    rc->mouse.x      = (int16_t)(buf[6]  | (buf[7]  << 8));
    rc->mouse.y      = (int16_t)(buf[8]  | (buf[9]  << 8));
    rc->mouse.z      = (int16_t)(buf[10] | (buf[11] << 8));
    rc->mouse.press_l = buf[12];
    rc->mouse.press_r = buf[13];
    rc->key.v         = (uint16_t)(buf[14] | (buf[15] << 8));

    rc->online = 1;  /* 解析成功即在线 */

    /* 记录最后接收时间戳 (本函数在 UART5 ISR 中调用, TIM6 时基) */
    s_rc_last_rx_ms = data_update_get_tick_ms();
}

void remote_control_init(void) {
    //memset(&remote_ctrl, 0, sizeof(remote_ctrl));
    RC_init();
}

const RC_ctrl_t *get_remote_control_point(void) { return &remote_ctrl; }

uint8_t RC_data_is_error(void) {
    if (!remote_ctrl.online) return 1;
    if (remote_ctrl.rc.s[0]==0 || remote_ctrl.rc.s[1]==0) return 1;
    return 0;
}

/* ================================================================
 *  remote_control_update — 在线超时检测 (主循环调用)
 *
 *  超过 RC_ONLINE_TIMEOUT_MS 未收到新帧 → online 清零.
 *  清零只在主循环进行, ISR 只在收到帧时置 1,
 *  超时窗口宽松, 竞态影响可忽略.
 * ================================================================ */
void remote_control_update(void)
{
    if (!remote_ctrl.online) return;

    if (data_update_get_tick_ms() - s_rc_last_rx_ms > RC_ONLINE_TIMEOUT_MS)
    {
        remote_ctrl.online = 0;
    }
}