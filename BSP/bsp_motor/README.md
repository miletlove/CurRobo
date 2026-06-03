# BSP/bsp_motor — CyberGear 电机驱动层

CAN 帧收发、MIT 模式组包、反馈帧解析。从 `Modules/cybergear/` 迁移至 BSP 层。

## 文件

| 文件 | 职责 |
|------|------|
| `cybergear_motor.h` | 数据结构定义 (`CyberGear_Motor_t`, `MITCmd_t`) + API 声明 |
| `cybergear_motor.c` | MIT 帧组包/解析, CAN 收发, 反馈回调链 |

## 协议支持

- 扩展帧 29-bit ID
- MIT 运控模式 (type=1): 位置/速度/KP/KD/力矩
- 反馈帧 (type=2): 角度/速度/力矩/温度/故障码
- 使能/停止/设零/运行模式切换

## API

```c
void     cg_motor_init(CyberGear_Motor_t *motor, uint8_t id, FDCAN_HandleTypeDef *hcan);
uint8_t  cg_motor_enable(CyberGear_Motor_t *motor);
uint8_t  cg_motor_stop(CyberGear_Motor_t *motor);
uint8_t  cg_motor_mit_control(CyberGear_Motor_t *motor, const CyberGear_MITCmd_t *cmd);
void     cg_motor_parse_feedback(uint32_t ext_id, const uint8_t *data, CyberGear_Feedback_t *fb);
```

## 依赖

- `BSP/bsp_can/` — FDCAN 发送/接收/中断
- `Core/Inc/fdcan.h` — FDCAN HAL 句柄
