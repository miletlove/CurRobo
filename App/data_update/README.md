# App/data_update — 系统数据更新模块

基于 **TIM6 1kHz 硬件定时器** 的频率调度与数据更新模块。

## 架构

```
TIM6 ISR (1kHz)
  → g_sys_tick_ms++          (系统 tick 计数器)
  → data_update_dispatch()   (频率调度, ISR 上下文)
    ├─ motor: 1kHz  → cg_ctrl_update_fixed() [ISR 安全 CAN 发送]
    ├─ imu:   200Hz → 置 g_flag_imu
    ├─ print: 1Hz   → 置 g_flag_print
    └─ led:   20Hz  → 置 g_flag_led

主循环 while(1)
  → data_update_execute()    (消费 ISR 标志, 安全上下文)
    ├─ BMI088_read()         [阻塞 SPI]
    ├─ usart1_print()        [阻塞 UART]
    └─ WS2812_Rainbow()      [时序敏感]
```

## ISR 安全设计

| 操作 | ISR 安全 | 原因 |
|------|---------|------|
| `HAL_FDCAN_AddMessageToTxFifoQ` | ✓ | 简单寄存器写入 |
| `cg_ctrl_update_fixed()` | ✓ | 控制量计算 + CAN 发送 |
| `HAL_SPI_TransmitReceive` (BMI088) | ✗ | 阻塞 SPI, timeout 1000ms |
| `usart1_print()` | ✗ | 阻塞 UART |
| `WS2812_Rainbow()` | ✗ | μs 级精确延时 |

## 频率配置

| 任务 | 频率 | tick 间隔 | 宏定义 |
|------|------|-----------|--------|
| 电机控制 | 1kHz | 1 | `DATA_UPDATE_FREQ_MOTOR = 1000` |
| IMU 读取 | 200Hz | 5 | `DATA_UPDATE_FREQ_IMU = 200` |
| 状态打印 | 1Hz | 1000 | `DATA_UPDATE_FREQ_PRINT = 1` |
| LED 刷新 | 20Hz | 50 | `DATA_UPDATE_FREQ_LED = 20` |

## API

```c
void     data_update_init(void);         // 启动 TIM6 1kHz 中断
uint32_t data_update_get_tick_ms(void);  // 获取系统毫秒计数 (替代 HAL_GetTick)
void     data_update_dispatch(void);     // ISR 内调用 — 频率调度入口
void     data_update_execute(void);      // 主循环调用 — 消费 ISR 标志
```

## 依赖

- TIM6: Prescaler=240-1, Period=1000-1 → 1kHz
- 电机控制: `cybergear_control.h`
- IMU: `BMI088driver.h`
- LED: `ws2812.h`
