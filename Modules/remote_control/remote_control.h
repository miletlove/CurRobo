#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "struct_typedef.h"

#define RC_FRAME_LENGTH     18u
#define RC_CH_VALUE_OFFSET  1024

/** 遥控器在线超时 (ms): 超过此时间未收到新帧 → 判定离线 */
#define RC_ONLINE_TIMEOUT_MS  500u

typedef enum { RC_SW_UP=1, RC_SW_MID=3, RC_SW_DOWN=2 } RC_Switch_t;
#define SW_IS_UP(s)    ((s)==RC_SW_UP)
#define SW_IS_MID(s)   ((s)==RC_SW_MID)
#define SW_IS_DOWN(s)  ((s)==RC_SW_DOWN)

typedef struct {
    uint8_t online;
    struct { int16_t ch[5]; RC_Switch_t s[2]; } rc;
    struct { int16_t x,y,z; uint8_t press_l,press_r; } mouse;
    struct { uint16_t v; } key;
} RC_ctrl_t;

/* 全局遥控器数据 (ISR 中更新, 主循环只读) */
extern RC_ctrl_t remote_ctrl;

/* SBUS 解析 (bsp_rc.c 的 ISR 中调用) */
void sbus_to_rc(const uint8_t *buf, RC_ctrl_t *rc);

/* App API */
void remote_control_init(void);
const RC_ctrl_t *get_remote_control_point(void);
uint8_t RC_data_is_error(void);

/**
 * @brief  遥控器在线状态周期更新 (主循环调用, TIM6 时基)
 * @note   超过 RC_ONLINE_TIMEOUT_MS 未收到新帧 → remote_ctrl.online 清零.
 *         需要在主循环每周期调用 (如 app_task_run 服务更新区).
 */
void remote_control_update(void);

#endif