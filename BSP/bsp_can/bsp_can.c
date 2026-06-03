/**
 * @file    bsp_can.c
 * @brief   FDCAN 板级支持包 (BSP) 实现
 * @note    基于 CtrBoard-H7_ALL/Bsp/bsp_can.c, 适配扩展帧 + CyberGear
 */
#include "bsp_can.h"
#include "bsp_usart.h"

/* ================================================================
 *  调试计数器 (ISR 中递增, 主循环读取)
 * ================================================================ */
volatile uint32_t g_dbg_can1_rx_cb_cnt = 0;  /* FDCAN1 回调计数 */
volatile uint32_t g_dbg_can2_rx_cb_cnt = 0;  /* FDCAN2 回调计数 */
volatile uint32_t g_dbg_can1_rx_irq_cnt = 0; /* FDCAN1 ISR 计数 */
volatile uint32_t g_dbg_fb_parsed_cnt = 0;   /* 反馈帧解析成功计数 */

volatile uint32_t g_dbg_rx_empty_cnt = 0;    /* can_bsp_receive 返回 0 次数 */
volatile uint32_t g_dbg_rx_got_cnt   = 0;    /* can_bsp_receive 返回 >0 次数 */
volatile uint32_t g_dbg_rx_first_ir  = 0;    /* 首次回调时 IR 寄存器值 */
volatile uint32_t g_dbg_rx_last_ir   = 0;    /* 最近一次 IR 寄存器值 */

/* ================================================================
 *  滤波器初始化 — 扩展帧, 初期接受全部 ID
 * ================================================================ */
void can_filter_init(void)
{
    FDCAN_FilterTypeDef fdcan_filter;

    /* ---- FDCAN1 滤波器 ---- */
    fdcan_filter.IdType       = FDCAN_EXTENDED_ID;
    fdcan_filter.FilterIndex  = 0;
    fdcan_filter.FilterType   = FDCAN_FILTER_MASK;
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filter.FilterID1    = 0x00000000;   /* 接受全部扩展帧 */
    fdcan_filter.FilterID2    = 0x00000000;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);

    /* ---- FDCAN2 滤波器 (与 FDCAN1 完全相同) ---- */
    fdcan_filter.FilterIndex  = 0;
    fdcan_filter.FilterID1    = 0x00000000;
    fdcan_filter.FilterID2    = 0x00000000;

    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);
}

/* ================================================================
 *  CAN 总线初始化
 * ================================================================ */
void can_bsp_init(void)
{
    /* 步骤1: 配置滤波器 (必须在 Start 之前, FDCAN 处于 INIT 模式) */
    can_filter_init();

    /* 步骤2: 配置中断线路由 — 必须在 Start 之前 (ILS 寄存器仅 INIT 模式可写)
     * FDCAN 有 IT0/IT1 两条中断线, 必须显式将 RX FIFO0 路由到 IT0,
     * 匹配 FDCANx_IT0_IRQHandler. */
    HAL_FDCAN_ConfigInterruptLines(&hfdcan1,
                                   FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                   FDCAN_INTERRUPT_LINE0);
    HAL_FDCAN_ConfigInterruptLines(&hfdcan2,
                                   FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                   FDCAN_INTERRUPT_LINE0);

    /* 步骤3: 启动 FDCAN1 & FDCAN2 (退出 INIT 模式, 进入 Normal 模式) */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
    {
        Error_Handler();
    }

    /* 步骤4: 使能接收中断通知 (IE 寄存器, Normal 模式可写) */
    HAL_FDCAN_ActivateNotification(&hfdcan1,
                                   FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan2,
                                   FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    /* ---- DEBUG: 回读寄存器, 合并为一次打印避免串口竞争 ---- */
    usart1_print("[DBG] ILS1=0x%08lX IE1=0x%08lX ILS2=0x%08lX IE2=0x%08lX NVIC_en=%d FIFO1=%lu FIFO2=%lu\r\n",
                 hfdcan1.Instance->ILS,
                 hfdcan1.Instance->IE,
                 hfdcan2.Instance->ILS,
                 hfdcan2.Instance->IE,
                 NVIC_GetEnableIRQ(FDCAN1_IT0_IRQn),
                 HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0),
                 HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0));
}

/*

*/
void can_power(uint8_t state)
{
    if(state == ENABLE)
    {
        HAL_GPIO_WritePin(POWER_OUT_1_GPIO_Port, POWER_OUT_1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(POWER_OUT_2_GPIO_Port, POWER_OUT_2_Pin, GPIO_PIN_SET);
    }
    if(state == DISABLE)
    {
        HAL_GPIO_WritePin(POWER_OUT_1_GPIO_Port, POWER_OUT_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(POWER_OUT_2_GPIO_Port, POWER_OUT_2_Pin, GPIO_PIN_RESET);
    }

}


/* ================================================================
 *  DLC 编码: 将字节长度映射为 HAL FDCAN DLC 宏
 *  Classic CAN: 0~8 直接映射
 *  FD CAN:     12/16/20/24/32/48/64 使用对应宏
 * ================================================================ */
static uint32_t can_dlc_encode(uint32_t len)
{
    switch (len)
    {
    case 0:  return FDCAN_DLC_BYTES_0;
    case 1:  return FDCAN_DLC_BYTES_1;
    case 2:  return FDCAN_DLC_BYTES_2;
    case 3:  return FDCAN_DLC_BYTES_3;
    case 4:  return FDCAN_DLC_BYTES_4;
    case 5:  return FDCAN_DLC_BYTES_5;
    case 6:  return FDCAN_DLC_BYTES_6;
    case 7:  return FDCAN_DLC_BYTES_7;
    case 8:  return FDCAN_DLC_BYTES_8;
    case 12: return FDCAN_DLC_BYTES_12;
    case 16: return FDCAN_DLC_BYTES_16;
    case 20: return FDCAN_DLC_BYTES_20;
    case 24: return FDCAN_DLC_BYTES_24;
    case 32: return FDCAN_DLC_BYTES_32;
    case 48: return FDCAN_DLC_BYTES_48;
    case 64: return FDCAN_DLC_BYTES_64;
    default: return FDCAN_DLC_BYTES_8;  /* 默认 8 字节 */
    }
}

/* ================================================================
 *  扩展帧发送
 * ================================================================ */
uint8_t can_bsp_send_extid(FDCAN_HandleTypeDef *hcan, uint32_t ext_id,
                           uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef TxHeader;

    TxHeader.Identifier          = ext_id;
    TxHeader.IdType              = FDCAN_EXTENDED_ID;
    TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
    TxHeader.DataLength          = can_dlc_encode(len);
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
    TxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker       = 0x00;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data) != HAL_OK)
        return 1;   /* 发送失败 */
    return 0;       /* 发送成功 */
}

/* ================================================================
 *  扩展帧接收 (非阻塞, 从 FIFO0 读取一帧)
 * ================================================================ */
uint8_t can_bsp_receive(FDCAN_HandleTypeDef *hcan, uint32_t *ext_id,
                        uint8_t *buf)
{
    FDCAN_RxHeaderTypeDef RxHeader;

    if (HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &RxHeader, buf) != HAL_OK)
        return 0;   /* 无数据 */

    *ext_id = RxHeader.Identifier;

    /* DLC 解码: HAL 返回的 DataLength 使用 FDCAN_DLC_BYTES_x 宏 */
    uint32_t dlc_code = RxHeader.DataLength;
    if (dlc_code <= FDCAN_DLC_BYTES_8)
        return (uint8_t)dlc_code;   /* 0~8 直接对应字节数 */
    if (dlc_code == FDCAN_DLC_BYTES_12) return 12;
    if (dlc_code == FDCAN_DLC_BYTES_16) return 16;
    if (dlc_code == FDCAN_DLC_BYTES_20) return 20;
    if (dlc_code == FDCAN_DLC_BYTES_24) return 24;
    if (dlc_code == FDCAN_DLC_BYTES_32) return 32;
    if (dlc_code == FDCAN_DLC_BYTES_48) return 48;
    if (dlc_code == FDCAN_DLC_BYTES_64) return 64;

    return 8;   /* 默认 */
}

/* ================================================================
 *  HAL FDCAN 中断回调 — 分发到 CAN1/CAN2 弱回调
 * ================================================================ */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0)
        return;

    /* DEBUG: 确认回调被调用 */
    if (hfdcan == &hfdcan1)
        g_dbg_can1_rx_cb_cnt++;
    else if (hfdcan == &hfdcan2)
        g_dbg_can2_rx_cb_cnt++;

    /* DEBUG: 直接读 IR 寄存器, 看是哪个中断源触发的 */
    uint32_t ir = hfdcan->Instance->IR;
    if (g_dbg_can1_rx_cb_cnt == 1 && hfdcan == &hfdcan1)
        g_dbg_rx_first_ir = ir;
    g_dbg_rx_last_ir = ir;

    uint32_t ext_id;
    uint8_t  rx_buf[8];
    uint8_t  len;

    if (hfdcan == &hfdcan1)
    {
        len = can_bsp_receive(&hfdcan1, &ext_id, rx_buf);
        if (len > 0)
        {
            g_dbg_rx_got_cnt++;
            can1_rx_callback(ext_id, rx_buf, len);
        }
        else
            g_dbg_rx_empty_cnt++;
    }
    else if (hfdcan == &hfdcan2)
    {
        len = can_bsp_receive(&hfdcan2, &ext_id, rx_buf);
        if (len > 0)
            can2_rx_callback(ext_id, rx_buf, len);
    }
}

/* ================================================================
 *  弱回调默认实现 (应用层应重写)
 * ================================================================ */
__weak void can1_rx_callback(uint32_t ext_id, uint8_t *data, uint8_t len)
{
    (void)ext_id;
    (void)data;
    (void)len;
    /* 默认空实现 — 在 cybergear_motor.c 中重写 */
}

__weak void can2_rx_callback(uint32_t ext_id, uint8_t *data, uint8_t len)
{
    (void)ext_id;
    (void)data;
    (void)len;
    /* 默认空实现 — 在 cybergear_motor.c 中重写 */
}
