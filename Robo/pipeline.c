/**
 * @file    pipeline.c
 * @brief   系统初始化管线实现 — 外设模块按依赖顺序逐一初始化
 * @author  CurRobo
 * @date    2026-06-02
 *
 * @note    替代原先散落在 main.c 和 data_update.c 中的 user_init().
 *          pipeline_init() 集中管理所有外设的启动顺序, 确保:
 *            - CAN 供电在 CAN 总线启动之前
 *            - CAN 总线在电机通信之前
 *            - TIM6 在所有模块就绪之后才启动
 */
#include "pipeline.h"
#include "data_update.h"
#include "cybergear_motor.h"
#include "cybergear_control.h"
#include "bsp_can.h"
#include "bsp_usart.h"
#include "BMI088driver.h"
#include "remote_control.h"

/* ================================================================
 *  外部引用 (main.c 中定义)
 * ================================================================ */
extern CyberGear_Motor_t    g_cg_motors[];
extern CyberGear_CtrlNode_t g_cg_ctrl[];
extern uint8_t              g_cg_motor_count;

/* ================================================================
 *  pipeline_init — 系统初始化管线
 *
 *  函数功能:
 *    按正确的依赖顺序初始化所有用户外设模块.
 *
 *  初始化顺序 (不可随意调整):
 *    1. USART1 打印 — 最早启用, 后续日志可输出
 *    2. 遥控器 DBUS — 依赖 USART1 DMA, 需在 CAN 前
 *    3. CAN 收发器供电 — POWER_OUT1/2 引脚拉高
 *    4. BMI088 IMU — 独立 SPI2, 可在 CAN 前或后
 *    5. CAN 总线 — 滤波器配置 → FDCAN_Start → 使能接收中断
 *    6. 电机初始化 — 绑定 FDCAN1 句柄 → 控制节点 → MIT 模式
 *    7. TIM6 1kHz — 最后启动, 确保所有模块就绪后才开始中断
 *
 *  调用时机:
 *    main() 中 MX_*_Init() 全部执行完毕后调用.
 *
 *  函数参数:
 *    无 (通过外部全局变量 g_cg_motors[], g_cg_ctrl[] 等访问)
 *
 *  函数输出:
 *    - 串口打印完整初始化日志
 *    - 所有外设就绪
 *    - TIM6 1kHz 中断开始运行
 */
void pipeline_init(void)
{
    /* ---- 1. 调试串口 ---- */
    // usart1_print("\r\n======== CurRobo ========\r\n");

    /* ---- 2. 遥控器 ---- */
    remote_control_init();

    /* ---- 3. CAN 收发器供电 ---- */
    can_power(ENABLE);

    /* ---- 4. BMI088 IMU ---- */
    // usart1_print("BMI088 init... ");
    // if (BMI088_init() == 0) usart1_print("OK\r\n");
    // else usart1_print("FAILED\r\n");
    BMI088_init();

    /* ---- 5. CAN 总线 ---- (FDCAN 调试信息在 can_bsp_init 内打印) */
    can_bsp_init();

    /* ---- 6. 电机初始化 ---- */
    for (uint8_t i = 0; i < g_cg_motor_count; i++)
    {
        cg_motor_init(&g_cg_motors[i], i + 1, &hfdcan1);
        cg_ctrl_init(&g_cg_ctrl[i], &g_cg_motors[i]);
    }

    /* ---- 7. TIM6 1kHz 定时器 (最后启动) ---- */
    data_update_init();
}
