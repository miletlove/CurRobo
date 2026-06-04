/**
 * @file    can_service.h
 * @brief   CAN 总线健康监控服务 — Bus-Off 检测 + 恢复
 * @author  CurRobo
 * @date    2026-06-05
 *
 * @note    监控 FDCAN1/2 的错误计数器和协议状态,
 *          当检测到 Bus-Off 时自动触发恢复流程.
 *
 *          关键寄存器:
 *            PSR (Protocol Status): ACT/EP/EW/BO
 *            ECR (Error Counter):   TEC/REC
 *
 *          Bus-Off 恢复流程:
 *            1. 检测 BO=1 且状态为 BUSY → 记录错误
 *            2. 等待 128×11 recessive bits (CAN 规范)
 *            3. HAL_FDCAN_Stop → 重置错误计数器 → HAL_FDCAN_Start
 */
#ifndef __CAN_SERVICE_H__
#define __CAN_SERVICE_H__

#include "main.h"
#include "fdcan.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  CAN 总线状态
 * ================================================================ */
typedef enum {
    CAN_STATE_OK       = 0,   /* 正常 */
    CAN_STATE_WARNING  = 1,   /* 错误警告 (TEC/REC ≥ 96) */
    CAN_STATE_PASSIVE  = 2,   /* 错误被动 (TEC/REC ≥ 128) */
    CAN_STATE_BUS_OFF  = 3    /* 总线关闭 (TEC ≥ 256) */
} CAN_BusState_t;

/* ================================================================
 *  CAN 总线健康信息
 * ================================================================ */
typedef struct {
    CAN_BusState_t state;
    uint8_t        tec;          /* 发送错误计数 */
    uint8_t        rec;          /* 接收错误计数 */
    uint8_t        active;       /* 总线活动标志 */
    uint32_t       rx_fifo_fill; /* Rx FIFO 填充数量 */
    uint32_t       last_error;   /* 最近错误码 (LEC) */
} CAN_Health_t;

/* ================================================================
 *  API
 * ================================================================ */

/**
 * @brief  CAN 服务初始化
 * @note   在 pipeline_init() 中 can_bsp_init() 之后调用
 */
void can_service_init(void);

/**
 * @brief  CAN 服务周期更新 — 检查健康状态 + 自动恢复
 * @note   在主循环中每 100ms 调用一次.
 * @retval 1=总线正常, 0=存在异常
 */
uint8_t can_service_update(void);

/**
 * @brief  获取指定 FDCAN 的健康信息
 * @param  bus  0=FDCAN1, 1=FDCAN2
 * @param  health [输出] 健康信息
 */
void can_service_get_health(uint8_t bus, CAN_Health_t *health);

/**
 * @brief  检查 FDCAN 总线是否正常
 */
uint8_t can_service_is_ok(uint8_t bus);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_SERVICE_H__ */
