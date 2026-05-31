# App — WS2812B RGB LED 驱动

## 硬件
- **SPI6 MOSI**: PA7 — 模拟 WS2812 单线时序
- 板载 RGB LED (DM-MC-board02)

## API
```c
void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b);  // 单颗灯珠立即刷新
void WS2812_SetColor(const WS2812_Color_t *color);   // 颜色结构体
void WS2812_Off(void);                                // 全灭
void WS2812_Rainbow(uint8_t speed);                   // 彩虹灯效 (非阻塞)
```

## 颜色预设
```c
WS2812_COLOR_RED, WS2812_COLOR_GREEN, WS2812_COLOR_BLUE
WS2812_COLOR_YELLOW, WS2812_COLOR_CYAN, WS2812_COLOR_MAGENTA
WS2812_COLOR_WHITE, WS2812_COLOR_ORANGE, WS2812_COLOR_OFF
```

## 使用示例
```c
WS2812_Ctrl(255, 0, 0);    // 红色
HAL_Delay(1000);
WS2812_Rainbow(3);         // 彩虹, speed=3
HAL_Delay(20);
```

## 注意事项
- SPI6 时钟 8MHz, 阻塞式 SPI 发送
- WS2812_Rainbow 需要周期性调用 (每 20ms)
- 时序通过 SPI 位编码模拟: 0码=0x60, 1码=0x78
