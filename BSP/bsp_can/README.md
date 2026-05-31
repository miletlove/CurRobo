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

## 初始化
```c
#include "bsp_can.h"

can_power(ENABLE);    // 上电 CAN 收发器
HAL_Delay(100);       // 等待稳定
can_bsp_init();       // 滤波器 + 启动 FDCAN1/2 + 使能中断
```

## 发送数据
```c
uint8_t data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
can_bsp_send_extid(&hfdcan1, 0x12345678, data, 8);  // 扩展帧 29-bit ID
```

## 接收数据 (中断回调)
```c
// 在 App 层重写弱回调:
void can1_rx_callback(uint32_t ext_id, uint8_t *data, uint8_t len) {
    // ext_id: 29-bit 扩展帧 ID
    // data: 8 字节数据
    // len: 接收长度
}
```
