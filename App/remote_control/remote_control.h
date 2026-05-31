#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "struct_typedef.h"

#define RC_FRAME_LENGTH     18u
#define RC_CH_VALUE_OFFSET  1024

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

#endif