/**
 * @file    bsp_rc.c
 * @brief   遥控器 DBUS — DMA 双缓冲 (DBM) 接收
 * @note    参照: 知乎「STM32H7系列教程2 DMA双缓冲区接收DT7遥控器」
 *          ISR 中直接调用 sbus_to_rc() 解析到 remote_ctrl
 */

#include "bsp_rc.h"
#include "main.h"
#include "usart.h"
#include "remote_control.h"
#include <string.h>

extern UART_HandleTypeDef huart5;

#define SBUS_RX_BUF_NUM     (RC_FRAME_LENGTH * 2u)   /* 36 */

/* 双缓冲区, 放在 DMA 可访问的 RAM_D1 (H7 DTCM 不可被 DMA 访问) */
static uint8_t sbus_buf[2][RC_FRAME_LENGTH] __attribute__((section(".dma_buffer"), aligned(32)));

static void USER_USART5_RxHandler(UART_HandleTypeDef *huart, uint16_t Size);

/* ================================================================
 *  RC_init
 * ================================================================ */
void RC_init(void)
{
    huart5.ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    huart5.RxEventType   = HAL_UART_RXEVENT_IDLE;
    huart5.RxXferSize    = SBUS_RX_BUF_NUM;

    SET_BIT(huart5.Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
    HAL_DMAEx_MultiBufferStart(huart5.hdmarx,
        (uint32_t)&huart5.Instance->RDR,
        (uint32_t)sbus_buf[0], (uint32_t)sbus_buf[1],
        SBUS_RX_BUF_NUM);
}

/* ================================================================
 *  HAL_UARTEx_RxEventCallback
 * ================================================================ */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart5)
        USER_USART5_RxHandler(huart, Size);
}

/* ================================================================
 *  USER_USART5_RxHandler — DBM 处理 + SBUS 解析
 *  CT=0 → M0AR 活跃 → buf[1] 有完整帧
 *  CT=1 → M1AR 活跃 → buf[0] 有完整帧
 * ================================================================ */
static void USER_USART5_RxHandler(UART_HandleTypeDef *huart, uint16_t Size)
{
    DMA_Stream_TypeDef *dma = (DMA_Stream_TypeDef *)huart->hdmarx->Instance;

    if ((dma->CR & DMA_SxCR_CT) == RESET)
    {
        __HAL_DMA_DISABLE(huart->hdmarx);

        if (Size == RC_FRAME_LENGTH)
        {
            SCB_InvalidateDCache_by_Addr((uint32_t *)sbus_buf[1], RC_FRAME_LENGTH);
            sbus_to_rc(sbus_buf[1], &remote_ctrl);
        }

        dma->CR |= DMA_SxCR_CT;
        __HAL_DMA_SET_COUNTER(huart->hdmarx, SBUS_RX_BUF_NUM);
    }
    else
    {
        __HAL_DMA_DISABLE(huart->hdmarx);

        if (Size == RC_FRAME_LENGTH)
        {
            SCB_InvalidateDCache_by_Addr((uint32_t *)sbus_buf[0], RC_FRAME_LENGTH);
            sbus_to_rc(sbus_buf[0], &remote_ctrl);
        }

        dma->CR &= ~(DMA_SxCR_CT);
        __HAL_DMA_SET_COUNTER(huart->hdmarx, SBUS_RX_BUF_NUM);
    }

    __HAL_DMA_ENABLE(huart->hdmarx);
}

/* ================================================================
 *  HAL_UART_ErrorCallback
 * ================================================================ */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart5) return;

    DMA_Stream_TypeDef *dma = (DMA_Stream_TypeDef *)huart->hdmarx->Instance;
    __HAL_DMA_DISABLE(huart->hdmarx);
    memset(sbus_buf, 0, sizeof(sbus_buf));
    dma->NDTR = SBUS_RX_BUF_NUM;
    __HAL_DMA_ENABLE(huart->hdmarx);
}


