# BSP — CAN 总线 (FDCAN Classic, 扩展帧)

## 硬件
| 信号 | MCU 引脚 |
|------|---------|
| FDCAN1 TX/RX | CubeMX 配置 |
| FDCAN2 TX/RX | CubeMX 配置 |
| CAN 收发器供电 | PC13 (POWER_OUT1), PC14 (POWER_OUT2) |

## CubeMX 配置
| 参数 | 值 |
|------|-----|
| Mode | FDCAN Classic |
| Bit Rate | 1 Mbps |
| Frame Format | Extended ID (29-bit) |
| NVIC | FDCAN1_IT0 / FDCAN2_IT0 **使能** |
| Interrupt Line | RX FIFO0 → Line 0 (必须调用 `HAL_FDCAN_ConfigInterruptLines`) |

> ⚠️ **H7 关键配置**: 必须在 `HAL_FDCAN_Start()` **之前**调用 `HAL_FDCAN_ConfigInterruptLines()` 将 RX FIFO0 中断路由到 IT0 线路。ILS 寄存器仅 INIT 模式可写。

## 初始化
```c
#include "bsp_can.h"

can_power(ENABLE);    // 上电 CAN 收发器
can_bsp_init();       // 滤波器 → ConfigInterruptLines → Start → ActivateNotification
```
> 正确顺序: Filter → `ConfigInterruptLines` (INIT模式) → `Start` (退出INIT) → `ActivateNotification`

## 发送数据
```c
uint8_t data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
can_bsp_send_extid(&hfdcan1, 0x12345678, data, 8);  // 扩展帧 29-bit ID
```

## 接收数据 (中断回调)
```c
// ⚠️ 在 App 层重写回调时, 头文件声明不使用 __weak.
// __weak 仅用于 bsp_can.c 的默认空定义, 否则所有包含该头文件的模块都会生成弱符号.
void can1_rx_callback(uint32_t ext_id, uint8_t *data, uint8_t len) {
    // ext_id: 29-bit 扩展帧 ID
    // data: 8 字节数据
    // len: 接收长度
}
```
