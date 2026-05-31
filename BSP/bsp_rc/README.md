# BSP — 遥控器 DBUS 接收 (DMA 双缓冲)

## 硬件连线
| DBUS 接口 | MCU 引脚 |
|-----------|---------|
| DBUS RX | PD2 (AF8) |
| 5V | 板载 5V 供电 |
| GND | 共地 |

## CubeMX 配置
| 参数 | 值 |
|------|-----|
| UART5 Mode | Asynchronous |
| Baud Rate | 100000 |
| Word Length | 9 Bits (including Parity) |
| Parity | Even |
| Stop Bits | 2 |
| DMA RX | DMA1_Stream0, **CIRCULAR**, P→M, VERY_HIGH |
| NVIC | UART5 global interrupt **使能** |

## 初始化
```c
#include "bsp_rc.h"

// 在 MX_UART5_Init() 之后调用
RC_init();  // 启动 DMA 双缓冲接收，之后自动在 ISR 中解析
```

## 使用方式
遥控器数据通过全局变量 `remote_ctrl` 访问（由 ISR 自动更新，无需轮询）：
```c
#include "remote_control.h"
const RC_ctrl_t *rc = get_remote_control_point();
if (!RC_data_is_error()) {
    int16_t ch0 = rc->rc.ch[0];   // 右摇杆左右 ±660
    int16_t ch2 = rc->rc.ch[2];   // 左摇杆上下 ±660
    // ...
}
```

## 关键注意事项
- **DMA 缓冲区必须在 RAM_D1 (0x24000000)**，H7 的 DTCM 不可被 DMA 访问
- 链接脚本需要 `.dma_buffer` 段 → `>RAM_D1`
- **必须用 CIRCULAR 模式**，NORMAL 模式下 HAL 会在首次 IDLE 后关闭 DMAR/IDLEIE
- H7 D-Cache：读 DMA 缓冲前必须 `SCB_InvalidateDCache_by_Addr`
