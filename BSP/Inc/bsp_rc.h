/**
 * @file    bsp_rc.h
 * @brief   遥控器 DBUS 接收 — DMA 双缓冲 (DBM)
 * @note    硬件: UART5_RX -> DMA1_Stream0, NORMAL, DBM
 */
#ifndef BSP_RC_H
#define BSP_RC_H

#include "struct_typedef.h"

void RC_init(void);

#endif
