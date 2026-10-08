/**
 * @file    id_test.c
 * @brief   FDCAN2 电机 ID 扫描测试实现
 * @author  CurRobo
 * @date    2026-10-09
 *
 * @note    复用 cybergear_motor 驱动接口, 帧格式与生产一致.
 *          时基: TIM6 (data_update_get_tick_ms), 不使用 SysTick.
 *          单个独立电机对象, 不触碰 g_cg_ctrl/g_cg_motors.
 */
#include "id_test.h"
#include "cybergear_motor.h"
#include "data_update.h"
#include "bsp_usart.h"

/* ================================================================
 *  被测总线句柄
 * ================================================================ */
#if ID_TEST_CAN == 1
#define ID_TEST_HCAN   (&hfdcan1)
#define ID_TEST_CAN_NAME "FDCAN1"
#else
#define ID_TEST_HCAN   (&hfdcan2)
#define ID_TEST_CAN_NAME "FDCAN2"
#endif

/* ================================================================
 *  测试参数
 * ================================================================ */
#define ID_TEST_PERIOD_MS       5000   /* 每 ID 测试周期 */
#define ID_TEST_STOP0_END_MS    1000   /* 阶段1: 初始 STOP */
#define ID_TEST_ENABLE_END_MS   2000   /* 阶段2: ENABLE */
#define ID_TEST_RUN_END_MS      4000   /* 阶段3: 1 rad/s 转圈 */
                                       /* 4000~5000: STOP 关闭使能 */
#define ID_TEST_TX_PERIOD_MS    100    /* 帧重发周期 (维持 MIT/确保送达) */
#define ID_TEST_VELOCITY_RAD    1.0f   /* 转圈速度 */
#define ID_TEST_KD              0.5f

/* ================================================================
 *  内部状态
 * ================================================================ */
static CyberGear_Motor_t s_motor;         /* 被测电机对象 (FDCAN2) */
static uint8_t           s_id = 1;        /* 当前测试 ID (1~4) */
static uint32_t          s_period_start_ms;

/* ================================================================
 *  切换到下一个 ID
 * ================================================================ */
static void id_test_next_id(void)
{
    s_id = (s_id % 4) + 1;
    cg_motor_init(&s_motor, s_id, ID_TEST_HCAN);
    s_period_start_ms = data_update_get_tick_ms();

    usart1_print("[IDTEST] ===> ID=%u begin (STOP->ENABLE->1rad/s->STOP)\r\n",
                 s_id);
}

/* ================================================================
 *  id_test_init
 * ================================================================ */
void id_test_init(void)
{
    s_id = 1;
    cg_motor_init(&s_motor, s_id, ID_TEST_HCAN);
    s_period_start_ms = data_update_get_tick_ms();

    usart1_print("\r\n======== Motor ID Test (%s) ========\r\n",
                 ID_TEST_CAN_NAME);
    usart1_print("[IDTEST] scan ID=1..4 on %s, 5s each, loop forever\r\n",
                 ID_TEST_CAN_NAME);
    usart1_print("[IDTEST] ===> ID=1 begin (STOP->ENABLE->1rad/s->STOP)\r\n");
}

/* ================================================================
 *  id_test_run — 主循环每周期调用
 * ================================================================ */
void id_test_run(void)
{
    uint32_t now   = data_update_get_tick_ms();
    uint32_t phase = now - s_period_start_ms;

    /* ---- 周期结束 → 下一个 ID ---- */
    if (phase >= ID_TEST_PERIOD_MS)
    {
        id_test_next_id();
        phase = 0;
    }

    /* ---- 每秒打印一次当前阶段 (便于与电机动作对照) ---- */
    static uint32_t last_sec = 0xFF;
    uint32_t sec = phase / 1000;
    if (sec != last_sec)
    {
        last_sec = sec;
        const char *stage;
        if (phase < ID_TEST_STOP0_END_MS)       stage = "STOP";
        else if (phase < ID_TEST_ENABLE_END_MS) stage = "ENABLE";
        else if (phase < ID_TEST_RUN_END_MS)    stage = "RUN 1rad/s";
        else                                    stage = "STOP(disable)";
        usart1_print("[IDTEST] ID=%u t=%lus %s\r\n", s_id, sec, stage);
    }

    /* ---- 周期发送当前阶段帧 (100ms 间隔) ---- */
    static uint32_t last_tx = 0;
    if (now - last_tx >= ID_TEST_TX_PERIOD_MS)
    {
        last_tx = now;

        if (phase < ID_TEST_STOP0_END_MS)
        {
            cg_motor_stop(&s_motor);
        }
        else if (phase < ID_TEST_ENABLE_END_MS)
        {
            cg_motor_enable(&s_motor);
        }
        else if (phase < ID_TEST_RUN_END_MS)
        {
            const CyberGear_MITCmd_t cmd = {
                .position = 0.0f,
                .velocity = ID_TEST_VELOCITY_RAD,
                .torque   = 0.0f,
                .kp       = 0.0f,
                .kd       = ID_TEST_KD,
            };
            cg_motor_mit_control(&s_motor, &cmd);
        }
        else
        {
            cg_motor_stop(&s_motor);   /* 停止 + 关闭使能 */
        }
    }
}
