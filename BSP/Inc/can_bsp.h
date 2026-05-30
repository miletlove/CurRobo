/**
 * @file    can_bsp.h
 * @brief   FDCAN 板级支持包 (BSP) — 扩展帧 CAN 通信接口
 * @note    适用于 STM32H723VGT6 + FDCAN1/FDCAN2 + CyberGear 电机
 *          使用扩展帧 29-bit ID, Classic CAN 模式, 1Mbps
 */
#ifndef __CAN_BSP_H__
#define __CAN_BSP_H__

#include "main.h"
#include "fdcan.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  API 函数
 * ================================================================ */

/**
 * @brief  CAN 总线初始化 (启动 FDCAN1/FDCAN2, 配置滤波器, 使能中断)
 */
void can_bsp_init(void);

/**
 * @brief  CAN 滤波器初始化 (扩展帧, 初期接受全部 ID)
 */
void can_filter_init(void);

/**
 * @brief CAN1/CAN2 收发器供电开启
 * @param state ENABLE=开启, DISABLE=关闭
 */
void can_power(uint8_t state);
/**
 * @brief  通过 FDCAN 发送扩展帧数据
 * @param  hcan    FDCAN 句柄 (&hfdcan1 或 &hfdcan2)
 * @param  ext_id  29-bit 扩展帧 ID
 * @param  data    发送数据缓冲区 (8 字节)
 * @param  len     数据长度 (固定 8 字节 for CyberGear)
 * @retval 0=成功, 1=发送失败
 */
uint8_t can_bsp_send_extid(FDCAN_HandleTypeDef *hcan, uint32_t ext_id,
                           uint8_t *data, uint32_t len);

/**
 * @brief  从 FDCAN FIFO0 接收扩展帧数据
 * @param  hcan    FDCAN 句柄
 * @param  ext_id  [输出] 29-bit 扩展帧 ID
 * @param  buf     接收数据缓冲区 (至少 8 字节)
 * @retval 接收到的数据长度 (0 表示无数据或接收失败)
 */
uint8_t can_bsp_receive(FDCAN_HandleTypeDef *hcan, uint32_t *ext_id,
                        uint8_t *buf);


/* ================================================================
 *  弱回调函数 — 应用层可重写
 * ================================================================ */

/**
 * @brief  FDCAN1 接收回调 (在 HAL 中断上下文中调用, 需保持简短)
 * @param  ext_id  接收到的扩展帧 ID
 * @param  data    接收数据
 * @param  len     数据长度
 */
__weak void can1_rx_callback(uint32_t ext_id, uint8_t *data, uint8_t len);

/**
 * @brief  FDCAN2 接收回调 (同上)
 */
__weak void can2_rx_callback(uint32_t ext_id, uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_BSP_H__ */
