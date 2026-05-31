# BSP — 遥控器 DBUS 接收 (DMA 双缓冲)

## 硬件
- **UART5_RX**: PD2 (AF8)
- **UART5_TX**: PC12 (AF8)
- **DMA**: DMA1_Stream0, CIRCULAR, P→M, VERY_HIGH
- **5V 供电**: DBUS 接口提供

## CubeMX 配置
| 参数 | 值 |
|------|-----|
| Baud Rate | 100000 |
| Word Length | 9 Bits (including Parity) |
| Parity | Even |
| Stop Bits | 2 |
| DMA Mode | **CIRCULAR** |
| NVIC | UART5 global interrupt **使能** |

## 工作原理
1. `RC_init()` 调用 `HAL_DMAEx_MultiBufferStart` 启动 DMA 双缓冲
2. DMA 自动在两个 18 字节缓冲区之间切换 (CT 位指示)
3. UART 检测到 IDLE (帧间隔) → ISR → `HAL_UARTEx_RxEventCallback`
4. 回调中根据 CT 位判断数据在哪个缓冲
5. `SCB_InvalidateDCache_by_Addr` 使 H7 D-Cache 失效
6. 调用 `sbus_to_rc()` 解析到全局 `remote_ctrl`

## 关键注意事项
- **DMA 缓冲区必须在 RAM_D1 (0x24000000)**，H7 的 DTCM 不可被 DMA 访问
- 链接脚本需要 `.dma_buffer` 段 → `>RAM_D1`
- **必须用 CIRCULAR 模式**，NORMAL 模式下 HAL 会在首次 IDLE 后关闭 DMAR/IDLEIE
- H7 D-Cache：读 DMA 缓冲前必须 `SCB_InvalidateDCache_by_Addr`

## 初始化
```c
#include "bsp_rc.h"
RC_init();  // 在 MX_UART5_Init() 之后调用
```
