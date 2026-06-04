/**
 * @file    can_service.c
 * @brief   CAN 总线健康监控服务实现
 * @author  CurRobo
 * @date    2026-06-05
 */
#include "can_service.h"
#include "bsp_can.h"
#include "data_update.h"
#include "bsp_usart.h"

/* ================================================================
 *  内部状态
 * ================================================================ */
static CAN_Health_t g_can_health[2];   /* 0=FDCAN1, 1=FDCAN2 */
static uint32_t     g_can_last_check_ms = 0;

/* ================================================================
 *  can_service_init
 * ================================================================ */
void can_service_init(void)
{
    for (uint8_t i = 0; i < 2; i++)
    {
        g_can_health[i].state       = CAN_STATE_OK;
        g_can_health[i].tec         = 0;
        g_can_health[i].rec         = 0;
        g_can_health[i].active      = 0;
        g_can_health[i].rx_fifo_fill= 0;
        g_can_health[i].last_error  = 0;
    }
}

/* ================================================================
 *  can_service_update — 每 100ms 检查一次
 * ================================================================ */
uint8_t can_service_update(void)
{
    uint32_t now = data_update_get_tick_ms();
    if (now - g_can_last_check_ms < 100) return 1;
    g_can_last_check_ms = now;

    uint8_t all_ok = 1;

    /* 检查 FDCAN1 */
    {
        uint32_t psr = hfdcan1.Instance->PSR;
        uint32_t ecr = hfdcan1.Instance->ECR;

        g_can_health[0].active = (psr & FDCAN_PSR_ACT) ? 1 : 0;
        g_can_health[0].tec    = (uint8_t)((ecr & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos);
        g_can_health[0].rec    = (uint8_t)((ecr & FDCAN_ECR_REC) >> FDCAN_ECR_REC_Pos);
        g_can_health[0].last_error = (psr & FDCAN_PSR_LEC);
        g_can_health[0].rx_fifo_fill = hfdcan1.Instance->RXF0S & FDCAN_RXF0S_F0FL;

        if (psr & FDCAN_PSR_BO)
        {
            g_can_health[0].state = CAN_STATE_BUS_OFF;
            usart1_print("[CAN] FDCAN1 Bus-Off! ECR=0x%08lX\r\n", ecr);
            /* 自动恢复: Stop → 重置错误计数器 → Start */
            HAL_FDCAN_Stop(&hfdcan1);
            HAL_FDCAN_Start(&hfdcan1);
            HAL_FDCAN_ActivateNotification(&hfdcan1,
                                           FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
            g_can_health[0].state = CAN_STATE_OK;
            usart1_print("[CAN] FDCAN1 recovered\r\n");
            all_ok = 0;
        }
        else if (g_can_health[0].tec >= 128 || g_can_health[0].rec >= 128)
        {
            g_can_health[0].state = CAN_STATE_PASSIVE;
            all_ok = 0;
        }
        else if (g_can_health[0].tec >= 96 || g_can_health[0].rec >= 96)
        {
            g_can_health[0].state = CAN_STATE_WARNING;
        }
        else
        {
            g_can_health[0].state = CAN_STATE_OK;
        }
    }

    /* 检查 FDCAN2 */
    {
        uint32_t psr = hfdcan2.Instance->PSR;
        uint32_t ecr = hfdcan2.Instance->ECR;

        g_can_health[1].active = (psr & FDCAN_PSR_ACT) ? 1 : 0;
        g_can_health[1].tec    = (uint8_t)((ecr & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos);
        g_can_health[1].rec    = (uint8_t)((ecr & FDCAN_ECR_REC) >> FDCAN_ECR_REC_Pos);
        g_can_health[1].last_error = (psr & FDCAN_PSR_LEC);

        if (psr & FDCAN_PSR_BO)
        {
            g_can_health[1].state = CAN_STATE_BUS_OFF;
            usart1_print("[CAN] FDCAN2 Bus-Off! ECR=0x%08lX\r\n", ecr);
            HAL_FDCAN_Stop(&hfdcan2);
            HAL_FDCAN_Start(&hfdcan2);
            HAL_FDCAN_ActivateNotification(&hfdcan2,
                                           FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
            g_can_health[1].state = CAN_STATE_OK;
            usart1_print("[CAN] FDCAN2 recovered\r\n");
            all_ok = 0;
        }
        else if (g_can_health[1].tec >= 128 || g_can_health[1].rec >= 128)
        {
            g_can_health[1].state = CAN_STATE_PASSIVE;
            all_ok = 0;
        }
        else if (g_can_health[1].tec >= 96 || g_can_health[1].rec >= 96)
        {
            g_can_health[1].state = CAN_STATE_WARNING;
        }
        else
        {
            g_can_health[1].state = CAN_STATE_OK;
        }
    }

    return all_ok;
}

/* ================================================================
 *  can_service_get_health
 * ================================================================ */
void can_service_get_health(uint8_t bus, CAN_Health_t *health)
{
    if (bus < 2 && health)
    {
        *health = g_can_health[bus];
    }
}

/* ================================================================
 *  can_service_is_ok
 * ================================================================ */
uint8_t can_service_is_ok(uint8_t bus)
{
    if (bus >= 2) return 0;
    return (g_can_health[bus].state == CAN_STATE_OK);
}
