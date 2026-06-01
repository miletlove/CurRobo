/**
 * @file    pipeline.h
 * @brief   系统初始化管线 — 统一封装所有外设模块的初始化顺序
 * @author  CurRobo
 * @date    2026-06-02
 *
 * @note    位于 Robo/ 目录, 与 BSP/ App/ Modules/ 同级.
 *          职责: 定义正确的硬件初始化顺序, 确保依赖关系正确.
 *
 *          初始化顺序:
 *            USART1 打印 → 遥控器 → CAN 供电 → BMI088 → CAN 总线
 *            → 电机对象 → MIT 模式 → TIM6 1kHz 定时器
 */
#ifndef __PIPELINE_H__
#define __PIPELINE_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  用户模块统一初始化 (在 main.c 中调用一次)
 * @note   封装所有外设模块的初始化顺序:
 *           1. USART1 调试串口 (最早, 便于调试)
 *           2. 遥控器 (DBUS)
 *           3. CAN 收发器供电 (POWER_OUT1/2, 在 CAN 总线启动前)
 *           4. BMI088 IMU
 *           5. CAN 总线 (滤波器 + FDCAN_Start + 中断使能)
 *           6. 电机对象绑定 + 控制节点初始化 + MIT 模式
 *           7. TIM6 1kHz 定时器 (最后启动, 避免初始化过程中触发中断)
 *         调用后所有模块就绪, 可进入主循环.
 */
void pipeline_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __PIPELINE_H__ */
