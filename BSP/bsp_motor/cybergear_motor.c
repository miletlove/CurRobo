/**
 * @file    cybergear_motor.c
 * @brief   CyberGear 电机驱动接口实现
 * @note    严格遵循 CyberData.md 协议:
 *          - 扩展帧 29-bit ID: (type<<24) | (data<<8) | id
 *          - MIT 模式: type=1, ID-data=力矩, CAN-data=位置/速度/KP/KD (大端序)
 *          - 反馈帧:   type=2, CAN-data=位置/速度/力矩/温度 (大端序)
 */
#include "cybergear_motor.h"
#include "bsp_can.h"

/* ================================================================
 *  内部辅助: 构造扩展帧 ID
 *  CyberData.md 二: bit24-28=mode, bit8-23=data, bit0-7=id
 * ================================================================ */
static inline uint32_t cg_build_extid(uint8_t type, uint8_t id,
                                      uint16_t data)
{
    return ((uint32_t)type << 24) | ((uint32_t)data << 8) | id;
}

/* ================================================================
 *  量化工具: float <-> uint16 (CyberData.md 六)
 * ================================================================ */
uint16_t cg_float_to_uint(float x, float x_min, float x_max)
{
    float span = x_max - x_min;
    if (x > x_max) x = x_max;
    if (x < x_min) x = x_min;
    return (uint16_t)((x - x_min) * 65535.0f / span);
}

float cg_uint_to_float(uint16_t x, float x_min, float x_max)
{
    return (float)x * (x_max - x_min) / 65535.0f + x_min;
}

/* ================================================================
 *  电机对象初始化
 * ================================================================ */
void cg_motor_init(CyberGear_Motor_t *motor, uint8_t id,
                   FDCAN_HandleTypeDef *hcan)
{
    motor->motor_id = id;
    motor->hcan     = hcan;
    motor->online   = 0;

    /* 清零反馈 */
    motor->feedback.position    = 0.0f;
    motor->feedback.velocity    = 0.0f;
    motor->feedback.torque      = 0.0f;
    motor->feedback.temperature = 0.0f;
    motor->feedback.mode_state  = 0;
    motor->feedback.fault       = 0;

}

/* ================================================================
 *  电机使能 (CyberData.md 十五)
 *  type=3, data 全零
 * ================================================================ */
uint8_t cg_motor_enable(CyberGear_Motor_t *motor)
{
    uint32_t ext_id = cg_build_extid(CG_TYPE_ENABLE, motor->motor_id, 0);
    uint8_t  data[8] = {0};
    return can_bsp_send_extid(motor->hcan, ext_id, data, 8);
}

/* ================================================================
 *  电机停止 (CyberData.md 十六)
 *  type=4, data 全零
 * ================================================================ */
uint8_t cg_motor_stop(CyberGear_Motor_t *motor)
{
    uint32_t ext_id = cg_build_extid(CG_TYPE_STOP, motor->motor_id, 0);
    uint8_t  data[8] = {0};
    return can_bsp_send_extid(motor->hcan, ext_id, data, 8);
}

/* ================================================================
 *  设置机械零位 (CyberData.md 三: type=6)
 * ================================================================ */
uint8_t cg_motor_set_zero(CyberGear_Motor_t *motor)
{
    uint32_t ext_id = cg_build_extid(CG_TYPE_SET_ZERO, motor->motor_id, 0);
    uint8_t  data[8] = {0};
    return can_bsp_send_extid(motor->hcan, ext_id, data, 8);
}

/* ================================================================
 *  MIT 运控模式控制 (CyberData.md 十四)
 *
 *  扩展 ID:
 *    mode = 1 (MIT)
 *    data = torque (uint16, 量化后)
 *    id   = motor_id
 *
 *  CAN DATA (8 字节, 大端序):
 *    Byte0-1: Position  (uint16)
 *    Byte2-3: Velocity  (uint16)
 *    Byte4-5: KP        (uint16)
 *    Byte6-7: KD        (uint16)
 * ================================================================ */
uint8_t cg_motor_mit_control(CyberGear_Motor_t *motor,
                             const CyberGear_MITCmd_t *cmd)
{
    /* 量化 float -> uint16 */
    uint16_t p_uint  = cg_float_to_uint(cmd->position, CG_P_MIN, CG_P_MAX);
    uint16_t v_uint  = cg_float_to_uint(cmd->velocity, CG_V_MIN, CG_V_MAX);
    uint16_t kp_uint = cg_float_to_uint(cmd->kp,       CG_KP_MIN, CG_KP_MAX);
    uint16_t kd_uint = cg_float_to_uint(cmd->kd,       CG_KD_MIN, CG_KD_MAX);
    uint16_t t_uint  = cg_float_to_uint(cmd->torque,   CG_T_MIN, CG_T_MAX);

    /* 大端序填充 DATA 区 (CyberData.md 七-1) */
    uint8_t data[8];
    data[0] = (uint8_t)(p_uint >> 8);
    data[1] = (uint8_t)(p_uint);
    data[2] = (uint8_t)(v_uint >> 8);
    data[3] = (uint8_t)(v_uint);
    data[4] = (uint8_t)(kp_uint >> 8);
    data[5] = (uint8_t)(kp_uint);
    data[6] = (uint8_t)(kd_uint >> 8);
    data[7] = (uint8_t)(kd_uint);

    /* 扩展 ID: type=1, data=力矩, id=电机ID */
    uint32_t ext_id = cg_build_extid(CG_TYPE_MIT_CTRL, motor->motor_id,
                                     t_uint);

    return can_bsp_send_extid(motor->hcan, ext_id, data, 8);
}

/* ================================================================
 *  设置运行模式 (CyberData.md 十七)
 *  type=18 (写参数), Index=0x7005 (小端序)
 * ================================================================ */
uint8_t cg_motor_set_run_mode(CyberGear_Motor_t *motor,
                              CyberGear_RunMode_t mode)
{
    uint32_t ext_id = cg_build_extid(CG_TYPE_WRITE_PARAM, motor->motor_id, 0);
    uint8_t  data[8] = {0};

    /* Index 0x7005 按 MCU 原始内存序 (小端序) (CyberData.md 七-2) */
    data[0] = 0x05;
    data[1] = 0x70;
    data[4] = (uint8_t)mode;

    return can_bsp_send_extid(motor->hcan, ext_id, data, 8);
}

/* ================================================================
 *  反馈帧解析 (CyberData.md 八)
 *
 *  扩展 ID:
 *    mode = 2
 *    data = fault 标志位 (bit16-21)
 *    id   = motor_id
 *
 *  CAN DATA (8 字节, 大端序):
 *    Byte0-1: 当前角度 (uint16 -> float)
 *    Byte2-3: 当前速度 (uint16 -> float)
 *    Byte4-5: 当前力矩 (uint16 -> float)
 *    Byte6-7: 当前温度 (uint16 -> float, 值=温度x10)
 *
 *  Warning: 此函数不做电机对象匹配，仅解析；匹配逻辑由调用方处理。
 * ================================================================ */
void cg_motor_parse_feedback(uint32_t ext_id, const uint8_t *data,
                             CyberGear_Feedback_t *fb)
{
    /* 大端序解析各 uint16 字段 */
    uint16_t p_raw   = ((uint16_t)data[0] << 8) | data[1];
    uint16_t v_raw   = ((uint16_t)data[2] << 8) | data[3];
    uint16_t t_raw   = ((uint16_t)data[4] << 8) | data[5];
    uint16_t temp_raw = ((uint16_t)data[6] << 8) | data[7];

    /* uint16 -> float 反量化 */
    fb->position    = cg_uint_to_float(p_raw,   CG_P_MIN, CG_P_MAX);
    fb->velocity    = cg_uint_to_float(v_raw,   CG_V_MIN, CG_V_MAX);
    fb->torque      = cg_uint_to_float(t_raw,   CG_T_MIN, CG_T_MAX);
    fb->temperature = (float)temp_raw * 0.1f;   /* 温度 x10 -> degree C */

    /* 从扩展 ID 提取 fault 和 mode (反馈帧专用布局) */
    /* Bit8-15: 电机 CAN ID | Bit16-21: 故障码 | Bit22-23: 模式状态 */
    fb->fault       = (uint8_t)((ext_id >> 16) & 0x3F);   /* bit16-21: 故障码    */
    fb->mode_state  = (uint8_t)((ext_id >> 22) & 0x03);   /* bit22-23: 模式状态  */
}

/* ================================================================
 *  重写 BSP 弱回调 — 在 CAN 中断中解析 CyberGear 反馈
 *
 *  用法: 将本文件与 can_bsp.c 链接后, can1/2_rx_callback 自动生效.
 *        用户在此处根据 ext_id 的 id 字段匹配对应电机对象并更新状态.
 * ================================================================ */

/* ---- 电机对象数组 (由 main.c 定义) ---- */
extern CyberGear_Motor_t g_cg_motors[];   /* 外部声明, 用户定义 */
extern uint8_t           g_cg_motor_count;

/**
 * @brief  根据电机 ID 和 CAN 总线查找电机对象
 * @return 找到返回指针, 否则返回 NULL
 */
static CyberGear_Motor_t *cg_find_motor(uint8_t motor_id,
                                        FDCAN_HandleTypeDef *hcan)
{
    for (uint8_t i = 0; i < g_cg_motor_count; i++)
    {
        if (g_cg_motors[i].motor_id == motor_id &&
            g_cg_motors[i].hcan == hcan)
        {
            return &g_cg_motors[i];
        }
    }
    return NULL;
}

/**
 * @brief  通用 CAN 接收处理 — 根据 type 字段分发, 更新电机对象
 *
 *         回调链路:
 *           FDCAN 中断 → HAL_FDCAN_RxFifo0Callback  [can_bsp.c]
 *                       → can1_rx_callback           [本文件重写]
 *                         → cg_rx_handler             [本函数]
 *                           → cg_motor_parse_feedback [解析 6 个字段]
 *                           → 更新 online / last_rx_tick / rx_count
 *                           → cg_motor_on_feedback    [弱回调, 应用层可重写]
 *
 *         函数参数:
 *           ext_id: 扩展帧 29-bit ID
 *                   bit28-24 = 通信类型 (0~31)
 *                   bit23-8  = 附加数据 (故障码 / 力矩等)
 *                   bit7-0   = 电机 CAN ID
 *           data:   8 字节 CAN 数据载荷
 *           len:    数据长度 (CyberGear 固定 8 字节)
 *           hcan:   收到此帧的 FDCAN 总线句柄
 *
 *         函数输出:
 *           匹配到的电机对象的 feedback 结构体被更新,
 *           online 置 1, last_rx_tick 和 rx_count 递增.
 *           同时触发 cg_motor_on_feedback 钩子.
 */
static void cg_rx_handler(uint32_t ext_id, uint8_t *data, uint8_t len,
                          FDCAN_HandleTypeDef *hcan)
{
    if (len < 8) return;   /* CyberGear 固定 8 字节 */

    uint8_t type     = (uint8_t)((ext_id >> 24) & 0x1F);   /* bit24-28: 通信类型 */
    uint8_t motor_id = (uint8_t)((ext_id >> 8) & 0xFF);    /* bit8-15:  电机 CAN ID */

    CyberGear_Motor_t *motor = cg_find_motor(motor_id, hcan);
    if (motor == NULL) return;

    switch (type)
    {
    case CG_TYPE_FEEDBACK:   /* type=2: 电机反馈 */
        cg_motor_parse_feedback(ext_id, data, &motor->feedback);
        motor->online       = 1;
        /* DEBUG: 反馈解析成功 */
        extern volatile uint32_t g_dbg_fb_parsed_cnt;
        g_dbg_fb_parsed_cnt++;
        cg_motor_on_feedback(motor);   /* 通知应用层 */
        break;

    case CG_TYPE_FAULT:      /* type=21: 故障反馈 */
        motor->feedback.fault = (uint8_t)((ext_id >> 8) & 0xFF);
        cg_motor_on_feedback(motor);
        break;

    default:
        break;
    }
}

/* ================================================================
 *  重写弱回调 — 链接时自动替换 BSP 中的空实现
 * ================================================================ */
void can1_rx_callback(uint32_t ext_id, uint8_t *data, uint8_t len)
{
    cg_rx_handler(ext_id, data, len, &hfdcan1);
}

void can2_rx_callback(uint32_t ext_id, uint8_t *data, uint8_t len)
{
    cg_rx_handler(ext_id, data, len, &hfdcan2);
}

/* ================================================================
 *  弱回调: 电机反馈通知 (应用层可在任意 .c 文件中重写)
 *
 *  调用时机: 每次收到电机 type=2 反馈帧或 type=21 故障帧后
 *  调用上下文: CAN 接收中断 (需保持简短, 禁止阻塞调用)
 *
 *  重写示例 (放在 main.c 中即可自动替换此空实现):
 *    void cg_motor_on_feedback(CyberGear_Motor_t *motor)
 *    {
 *        if (motor->feedback.fault != 0)
 *        {
 *            // 电机故障! 执行紧急停止
 *            cg_motor_stop(motor);
 *        }
 *    }
 * ================================================================ */
__weak void cg_motor_on_feedback(CyberGear_Motor_t *motor)
{
    (void)motor;
    /* 默认空实现 — 应用层可重写 */
}
