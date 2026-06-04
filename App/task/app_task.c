/**
 * @file    app_task.c
 * @brief   应用任务调度 — 启动 FSM + 服务调度 + 命令源路由
 * @author  CurRobo
 * @date    2026-06-05
 *
 * @note    启动状态机:
 *          ┌─────────────────────────────────────────────┐
 *          │  APP_FSM_INIT                                │
 *          │    → 首次使能已完成, 等待电机上线            │
 *          │    → motor_service_update() 自动重试         │
 *          │    → 全在线 → APP_FSM_READY                  │
 *          │    → 超时   → APP_FSM_FAULT                  │
 *          │                                              │
 *          │  APP_FSM_READY                               │
 *          │    → 正常控制: wheel_update()                │
 *          │    → 电机掉线 → APP_FSM_STARTING             │
 *          │    → 遥控器离线 → 全停 (保持 READY)           │
 *          │                                              │
 *          │  APP_FSM_FAULT                               │
 *          │    → 等待用户操作清除故障                    │
 *          │    → 遥控器拨杆清除 → APP_FSM_STARTING       │
 *          └─────────────────────────────────────────────┘
 *
 *          命令源路由 (为上位机预留):
 *            当前: CMD_SOURCE_RC (遥控器)
 *            未来: CMD_SOURCE_UPPER (上位机串口/CAN)
 *            MotionCommand_t 统一接口
 */
#include "app_task.h"
#include "data_update.h"
#include "wheel.h"
#include "motor_service.h"
#include "can_service.h"
#include "remote_control.h"
#include "bsp_usart.h"

#include "watchdog.h"

/* ================================================================
 *  内部状态
 * ================================================================ */

/* 前置声明 */
static void app_task_fsm_update(void);
static void app_task_wheel_control(void);

/** 当前 FSM 状态 */
static AppFsmState_t g_fsm_state = APP_FSM_INIT;

/** FSM 状态进入时间戳 (用于超时判断) */
static uint32_t g_fsm_entry_ms = 0;

/** 启动超时 (ms): 超过此时间电机未全在线则进入 FAULT */
#define APP_STARTUP_TIMEOUT_MS   10000   /* 10 秒 */

/** 上一次状态打印时间 (避免刷屏) */
static uint32_t g_last_state_print = 0;

/* ================================================================
 *  内部辅助
 * ================================================================ */

/** FSM 状态转换 */
static void app_fsm_transition(AppFsmState_t new_state)
{
    if (g_fsm_state == new_state) return;

    const char *names[] = { "INIT", "STARTING", "READY", "FAULT" };
    usart1_print("[FSM] %s → %s\r\n",
                 names[g_fsm_state], names[new_state]);

    g_fsm_state       = new_state;
    g_fsm_entry_ms    = data_update_get_tick_ms();
    g_last_state_print = 0;
}

/* ================================================================
 *  app_task_init — 应用任务统一初始化
 * ================================================================ */
void app_task_init(void)
{
    /* 电机: 配置速度模式 + 首次使能 */
    wheel_init();                    /* 设置速度模式 */
    motor_service_init();            /* 绑定控制节点 */
    motor_service_enable_all();      /* 发送使能帧 */
    can_service_init();              /* CAN 健康监控 */

    /* IWDG: CubeMX 配置后取消注释 */
    // watchdog_init();

    app_fsm_transition(APP_FSM_INIT);

    usart1_print("[FSM] Startup begin — waiting for motors...\r\n");
}

/* ================================================================
 *  app_task_run — 应用任务统一执行 (主循环每周期)
 * ================================================================ */
void app_task_run(void)
{
    /* ---- 1. 消费 TIM6 ISR 标志 (IMU/打印/LED) ---- */
    data_update_execute();

    /* ---- 2. 服务更新 ---- */
    motor_service_update();   /* 电机重试 + 健康检查 */
    can_service_update();     /* CAN 总线健康监控 */

    /* ---- 3. 启动状态机 ---- */
    app_task_fsm_update();

    /* ---- 4. 控制执行 (仅 READY 态) ---- */
    if (g_fsm_state == APP_FSM_READY)
    {
        app_task_wheel_control();
    }

    /* ---- 5. IWDG 看门狗: 由 TIM6 ISR (data_update.c) 喂狗, 主循环不喂 ---- */
    /* watchdog_feed() 调用位置在 data_update.c 的 HAL_TIM_PeriodElapsedCallback 末尾 */
}

/* ================================================================
 *  app_task_fsm_update — 启动状态机
 * ================================================================ */
void app_task_fsm_update(void)
{
    uint32_t now = data_update_get_tick_ms();
    uint8_t all_online = motor_service_all_online();

    switch (g_fsm_state)
    {
    case APP_FSM_INIT:
        if (all_online)
        {
            app_fsm_transition(APP_FSM_READY);
        }
        else if (now - g_fsm_entry_ms > APP_STARTUP_TIMEOUT_MS)
        {
            usart1_print("[FSM] Startup timeout (%lu ms), entering FAULT\r\n",
                         APP_STARTUP_TIMEOUT_MS);
            app_fsm_transition(APP_FSM_FAULT);
        }
        /* 每秒打印一次启动进度 */
        if (now - g_last_state_print > 1000)
        {
            g_last_state_print = now;
            usart1_print("[FSM] Waiting... M1=%s M2=%s M3=%s M4=%s\r\n",
                         motor_service_get_state(0) == MOTOR_STATE_ONLINE ? "ON" : "--",
                         motor_service_get_state(1) == MOTOR_STATE_ONLINE ? "ON" : "--",
                         motor_service_get_state(2) == MOTOR_STATE_ONLINE ? "ON" : "--",
                         motor_service_get_state(3) == MOTOR_STATE_ONLINE ? "ON" : "--");
        }
        break;

    case APP_FSM_READY:
        /* 运行时掉线 → 回到 INIT 重试 */
        if (!all_online)
        {
            usart1_print("[FSM] Motor offline, re-entering startup\r\n");
            app_fsm_transition(APP_FSM_INIT);
        }
        break;

    case APP_FSM_FAULT:
        /* 故障恢复: 当所有电机重新在线时自动恢复 */
        if (all_online)
        {
            app_fsm_transition(APP_FSM_READY);
        }
        /* 或在遥控器特定操作时手动恢复 (预留) */
        /* 每秒打印故障状态 */
        if (now - g_last_state_print > 1000)
        {
            g_last_state_print = now;
            usart1_print("[FSM] FAULT — check motor power/CAN connection\r\n");
        }
        break;

    default:
        break;
    }
}

/* ================================================================
 *  app_task_wheel_control — 遥控器 → 四轮速度 (仅 READY 态)
 * ================================================================ */
void app_task_wheel_control(void)
{
    /* ── 安全保护 1: 遥控器离线 ── */
    if (!remote_ctrl.online)
    {
        motor_service_soft_stop();
        return;
    }

    /* ── 安全保护 2: 电机全部在线 ── */
    if (!motor_service_all_online())
    {
        motor_service_soft_stop();
        return;
    }

    /* ── 正常: 遥控器 → 四轮速度映射 ── */
    wheel_update();
}

/* ================================================================
 *  app_task_get_fsm_state
 * ================================================================ */
AppFsmState_t app_task_get_fsm_state(void)
{
    return g_fsm_state;
}

/* ================================================================
 *  app_task_rc_to_command — 遥控器 → MotionCommand (预留)
 *
 *  当前由 wheel_update() 内部直接读取 remote_ctrl.
 *  未来上位机指令接入时, 可用此函数统一生成 MotionCommand,
 *  wheel_update() 改为 wheel_update_from_cmd(&cmd).
 * ================================================================ */
#if 0  /* 预留: 上位机指令接口启用时改为 1 */
static void app_task_rc_to_command(MotionCommand_t *cmd)
{
    cmd->vx     = (float)remote_ctrl.rc.ch[0] / 660.0f * 2.0f;
    cmd->vy     = (float)remote_ctrl.rc.ch[1] / 660.0f * 2.0f;
    cmd->wz     = 0.0f;
    cmd->source = CMD_SOURCE_RC;
    cmd->valid  = remote_ctrl.online;
}
#endif
