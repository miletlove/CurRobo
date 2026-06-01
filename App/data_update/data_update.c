/**
 * @file    data_update.c
 * @brief   系统数据更新模块实现 — TIM6 ISR 频率调度 + 主循环安全执行
 * @author  CurRobo
 * @date    2026-06-01
 *
 * @note    TIM6 配置: Prescaler=240-1, Period=1000-1 → 1kHz (1ms)
 *
 *          ISR 安全分析:
 *          ─────────────────────────────────────────────────────
 *          ✓ ISR 安全 (可在 TIM6 中断中调用):
 *            - HAL_FDCAN_AddMessageToTxFifoQ  (简单 Tx FIFO 写入)
 *            - cg_ctrl_update_fixed()          (控制量计算 + CAN 发送)
 *            - GPIO 操作
 *
 *          ✗ 非 ISR 安全 (必须在主循环中调用):
 *            - HAL_SPI_TransmitReceive (BMI088_read 阻塞 SPI)
 *            - usart1_print             (阻塞 UART)
 *            - WS2812_Rainbow           (时序敏感, μs 级)
 *            - HAL_Delay                (阻塞)
 *          ─────────────────────────────────────────────────────
 */
#include "data_update.h"
#include "cybergear_control.h"
#include "BMI088driver.h"
#include "ws2812.h"
#include "bsp_usart.h"

/* ================================================================
 *  全局变量
 * ================================================================ */

/** 系统毫秒计数器 (TIM6 中断递增) */
static volatile uint32_t g_sys_tick_ms = 0;

/** 各任务执行标志 (ISR 置 1, 主循环消费后清 0) */
static volatile uint8_t  g_flag_imu   = 0;
static volatile uint8_t  g_flag_print = 0;
static volatile uint8_t  g_flag_led   = 0;

/* ================================================================
 *  外部引用
 * ================================================================ */
extern CyberGear_Motor_t    g_cg_motors[];
extern CyberGear_CtrlNode_t g_cg_ctrl[];
extern uint8_t              g_cg_motor_count;
extern float                gyro[3], accel[3], temp;

/* ================================================================
 *  内部: 各任务频率调度函数 (在 ISR 中调用, 自行判断是否到期)
 *
 *  设计模式:
 *    static uint32_t last = 0;
 *    uint32_t now = g_sys_tick_ms;
 *    if (now - last >= TICK_INTERVAL) {
 *        last = now;
 *        执行业务逻辑 (ISR 安全) 或 置标志 (非 ISR 安全)
 *    }
 * ================================================================ */

/**
 * @brief  电机控制任务 (1kHz, ISR 内直接执行)
 *
 *  ISR 安全性: ✓
 *    cg_ctrl_update_fixed() 内部调用 cg_motor_mit_control()
 *    → can_bsp_send_extid() → HAL_FDCAN_AddMessageToTxFifoQ()
 *    FDCAN Tx FIFO 写入是简单寄存器操作, STM32H7 HAL 文档确认 ISR 安全.
 */
static void data_update_task_motor(void)
{
    static uint32_t last = 0;
    uint32_t now = g_sys_tick_ms;
    if (now - last < DATA_UPDATE_TICK_MOTOR) return;
    last = now;

    for (uint8_t i = 0; i < g_cg_motor_count; i++)
    {
        cg_ctrl_sync_online(&g_cg_ctrl[i]);
        if (!g_cg_ctrl[i].online || !g_cg_ctrl[i].enabled) continue;
        cg_ctrl_update_fixed(&g_cg_ctrl[i], 0.001f);  /* dt = 1ms */
    }
}

/**
 * @brief  IMU 读取任务 (200Hz, 置标志 → 主循环执行)
 *
 *  ISR 安全性: ✗ (BMI088_read 使用阻塞 HAL_SPI_TransmitReceive)
 *  因此在 ISR 中仅做频率判断 + 置标志.
 */
static void data_update_task_imu(void)
{
    static uint32_t last = 0;
    uint32_t now = g_sys_tick_ms;
    if (now - last < DATA_UPDATE_TICK_IMU) return;
    last = now;
    g_flag_imu = 1;
}

/**
 * @brief  状态打印任务 (1Hz, 置标志 → 主循环执行)
 *
 *  ISR 安全性: ✗ (usart1_print 使用阻塞 HAL_UART_Transmit)
 */
static void data_update_task_print(void)
{
    static uint32_t last = 0;
    uint32_t now = g_sys_tick_ms;
    if (now - last < DATA_UPDATE_TICK_PRINT) return;
    last = now;
    g_flag_print = 1;
}

/**
 * @brief  LED 刷新任务 (20Hz, 置标志 → 主循环执行)
 *
 *  ISR 安全性: ✗ (WS2812 依赖精确 μs 延时, 中断会破坏时序)
 */
static void data_update_task_led(void)
{
    static uint32_t last = 0;
    uint32_t now = g_sys_tick_ms;
    if (now - last < DATA_UPDATE_TICK_LED) return;
    last = now;
    g_flag_led = 1;
}

/* ================================================================
 *  HAL 弱回调重写 — TIM6 周期中断回调
 *
 *  ⚠️ 此函数运行在 TIM6 中断上下文中 (优先级 1,0).
 *     内部仅做: tick++ → dispatch 各任务.
 *     dispatch 中 ISR 安全的直接执行, 不安全的置标志.
 * ================================================================ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        g_sys_tick_ms++;
        data_update_dispatch();
    }
}

/* ================================================================
 *  API 实现
 * ================================================================ */

void data_update_init(void)
{
    g_sys_tick_ms = 0;
    g_flag_imu    = 0;
    g_flag_print  = 0;
    g_flag_led    = 0;

    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        Error_Handler();
    }
}

uint32_t data_update_get_tick_ms(void)
{
    return g_sys_tick_ms;
}

/**
 * @brief  TIM6 ISR 内调用 — 统一频率调度
 *
 *  函数功能:
 *    根据 g_sys_tick_ms 判断各任务是否到达执行周期:
 *      - motor: 每 1 tick  (1kHz)   → 直接执行 (ISR 安全 CAN 发送)
 *      - imu:   每 5 ticks (200Hz)  → 置 g_flag_imu
 *      - print: 每 1000 ticks (1Hz) → 置 g_flag_print
 *      - led:   每 50 ticks (20Hz)  → 置 g_flag_led
 *
 *  调用上下文:
 *    HAL_TIM_PeriodElapsedCallback → ISR 上下文.
 *    本函数内仅允许 ISR 安全的操作.
 */
void data_update_dispatch(void)
{
    data_update_task_motor();
    data_update_task_imu();
    data_update_task_print();
    data_update_task_led();
}

/**
 * @brief  主循环中调用 — 消费 ISR 设置的标志, 在安全上下文中执行
 *
 *  函数功能:
 *    检查并消费各任务标志:
 *      - g_flag_imu   → BMI088_read()     [SPI 阻塞]
 *      - g_flag_print → 状态打印           [UART 阻塞]
 *      - g_flag_led   → WS2812_Rainbow()   [非 ISR 安全]
 *
 *  调用上下文:
 *    主循环 while(1) 中, 非 ISR 上下文.
 *    所有阻塞操作 (SPI/UART/HAL_Delay) 在此安全执行.
 *
 *  函数参数:
 *    无 (通过外部全局变量访问)
 *
 *  函数输出:
 *    - gyro[], accel[], temp 更新为最新 IMU 数据
 *    - 串口输出当前状态
 *    - WS2812 灯效刷新
 */
void data_update_execute(void)
{
    uint32_t now = g_sys_tick_ms;

    /* ---- IMU 读取 (200Hz) ---- */
    if (g_flag_imu)
    {
        g_flag_imu = 0;
        BMI088_read(gyro, accel, &temp);
    }

    /* ---- 状态打印 (1Hz) ---- */
    if (g_flag_print)
    {
        g_flag_print = 0;
        usart1_print("[%5lu] | "
                     "M0 p=%.2f v=%.2f t=%.2f temp=%.1f | "
                     "IMU gz=%.2f\r\n",
                     now,
                     g_cg_motors[0].feedback.position,
                     g_cg_motors[0].feedback.velocity,
                     g_cg_motors[0].feedback.torque,
                     g_cg_motors[0].feedback.temperature,
                     gyro[2]);
    }

    /* ---- LED 刷新 (20Hz) ---- */
    if (g_flag_led)
    {
        g_flag_led = 0;
        WS2812_Rainbow(3);
    }
}
