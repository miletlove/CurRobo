# BSP — CAN 总线 (FDCAN 扩展帧)

## 硬件
- **FDCAN1** / **FDCAN2**
- POWER_OUT1 (PC13) / POWER_OUT2 (PC14): CAN 收发器供电控制

## CubeMX 配置
| 参数 | 值 |
|------|-----|
| Mode | FDCAN Classic |
| Bit Rate | 1 Mbps |
| Frame Format | Extended ID (29-bit) |
| NVIC | FDCAN1_IT0 / FDCAN2_IT0 **使能** |

## API
```c
void    can_bsp_init(void);            // 初始化 (滤波器 + 启动 + 中断)
void    can_power(uint8_t state);      // CAN 收发器供电 (ENABLE/DISABLE)
uint8_t can_bsp_send_extid(hcan, ext_id, data, len);  // 发送扩展帧
uint8_t can_bsp_receive(hcan, &ext_id, buf);          // 接收扩展帧
```

## 初始化
```c
can_power(ENABLE);
HAL_Delay(100);
can_bsp_init();
```

## 中断回调
```c
// 弱回调, 应用层重写:
void can1_rx_callback(uint32_t ext_id, uint8_t *data, uint8_t len);
void can2_rx_callback(uint32_t ext_id, uint8_t *data, uint8_t len);
```
