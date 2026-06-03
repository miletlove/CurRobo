# Robo/wheel — 轮式运动模块

遥控器摇杆通道 → 电机速度线性映射。

## 映射公式

$$\omega_{des} = \frac{ch_1}{660} \times 1.0\ \text{rad/s}$$

- ch1 > 0 → 电机正转, ch1 < 0 → 反转
- 超范围自动钳位至 ±1.0 rad/s
- 速度阻尼 Kd=0.6 (Nm/(rad/s))

## 配置宏

| 宏 | 值 | 说明 |
|----|-----|------|
| `WHEEL_RC_CH_MAX_ABS` | 660 | 摇杆通道最大偏移 |
| `WHEEL_MAX_SPEED_RAD_S` | 1.0f | 最大转速 (rad/s) |
| `WHEEL_KD_VELOCITY` | 0.6f | 速度阻尼系数 |

## API

```c
void wheel_init(void);    // 配置电机为速度模式 + 使能
void wheel_update(void);  // 读取摇杆 → 更新目标速度
```

## 依赖

- `Modules/cybergear/` — 控制层 (速度模式)
- `Modules/remote_control/` — 遥控器数据
