/**
 * @file    id_test.h
 * @brief   FDCAN2 电机 ID 扫描测试 — 逐 ID (1~4) 执行启停动作序列
 * @author  CurRobo
 * @date    2026-10-09
 *
 * @note    测试目标:
 *           确认 FDCAN2 上各电机实际的 CAN ID.
 *           依次对 ID=1,2,3,4 发送测试序列, 观察哪台电机转动,
 *           即可确认每台电机的真实 ID.
 *
 *          每 ID 测试周期 5s (TIM6 时基):
 *            0~1s:   STOP 帧 (确保初始停止)
 *            1~2s:   ENABLE 帧 (使能)
 *            2~4s:   MIT 帧, 速度 1.0 rad/s (转圈)
 *            4~5s:   STOP 帧 (停止并关闭使能)
 *            → 下一个 ID, 4 个 ID 循环
 *
 *          测试期间跳过 app_task (电机自动使能/控制逻辑),
 *          串口仅输出 [IDTEST] 日志.
 */
#ifndef __ID_TEST_H__
#define __ID_TEST_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  测试模式开关: 1=运行 ID 扫描测试, 0=正常运行逻辑
 * ================================================================ */
#define ID_SCAN_TEST  0

/* ================================================================
 *  被测总线: 1=FDCAN1, 2=FDCAN2
 * ================================================================ */
#define ID_TEST_CAN   1

void id_test_init(void);
void id_test_run(void);

#ifdef __cplusplus
}
#endif

#endif /* __ID_TEST_H__ */
