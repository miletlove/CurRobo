/**
 * @file    remote_control.c
 * @brief   遥控器数据管理 — SBUS 解析 + 全局数据
 * @note    remote_ctrl 由 bsp_rc.c 的 ISR 直接更新, 主循环只读
 */
#include "remote_control.h"
#include "bsp_rc.h"
#include <string.h>

/* 全局遥控器数据 (ISR 写入, 主循环读取) */
RC_ctrl_t remote_ctrl;

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