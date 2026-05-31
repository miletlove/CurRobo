# BSP — USART1 串口打印 (阻塞发送)

## 硬件
- **USART1_TX**: PA9 (AF7)
- **USART1_RX**: PA10 (AF7)

## CubeMX 配置
| 参数 | 值 |
|------|-----|
| Baud Rate | 115200 |
| Word Length | 8 Bits |
| Parity | None |
| Stop Bits | 1 |
| DMA | 不需要 (阻塞发送) |

## API
```c
void usart1_send(const uint8_t *data, uint16_t len);  // 阻塞发送原始字节
void usart1_print(const char *fmt, ...);               // 阻塞格式化打印
```

## 使用示例
```c
#include "bsp_usart.h"
usart1_print("Hello %d\r\n", 42);
usart1_send((uint8_t *)"OK\r\n", 4);
```

## 注意事项
- 阻塞发送，发送完成前函数不返回
- 内部缓冲 256 字节，超长截断
- 不依赖 DMA，简单可靠
