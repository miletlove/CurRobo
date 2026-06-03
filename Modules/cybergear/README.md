# Modules — CyberGear 电机控制库 (精简版)

| 文件 | 层级 | 职责 |
|------|------|------|
| `cybergear_control.h/.c` | **控制层** | 阻抗控制、速度控制 |
| ~~`cybergear_motor.h/.c`~~ | → 已迁移至 `BSP/bsp_motor/` | CAN 帧收发、MIT 模式组包、反馈解析 |

## 控制模式

| 模式 | 枚举 | 公式 | 适用场景 |
|------|------|------|---------|
| 空闲 | `CG_CTRL_MODE_IDLE` | 不下发指令 | 未使能 |
| 阻抗控制 | `CG_CTRL_MODE_IMPEDANCE` | $\tau = K(\theta_d-\theta) + D(\omega_d-\omega) + \tau_{ff}$ | 柔顺着陆、顺应地形 |
| 速度控制 | `CG_CTRL_MODE_VELOCITY` | $\tau = K_d(\omega_d-\omega) + \tau_{ff}$ | 轮式运动 |

## API
```c
void cg_ctrl_init(CyberGear_CtrlNode_t *ctrl, CyberGear_Motor_t *motor);
void cg_ctrl_set_impedance(CyberGear_CtrlNode_t *ctrl, float stiffness, float damping);
void cg_ctrl_set_velocity(CyberGear_CtrlNode_t *ctrl, float kd);
void cg_ctrl_set_target(CyberGear_CtrlNode_t *ctrl, float pos, float vel, float torque_ff);
uint8_t cg_ctrl_enable(CyberGear_CtrlNode_t *ctrl);
uint8_t cg_ctrl_stop(CyberGear_CtrlNode_t *ctrl);
uint8_t cg_ctrl_update_fixed(CyberGear_CtrlNode_t *ctrl, float dt);
```

> **注意**: `online` 仅用于状态监控, 不阻塞 MIT 帧发送。电机需要持续 MIT 帧才能维持反馈, 阻塞会导致死锁。

### 数据结构
```c
CyberGear_Motor_t motor;
cg_motor_init(&motor, motor_id, &hfdcan1);  // motor_id: 0x01~0x7F
```

### API
```c
void    cg_motor_init(CyberGear_Motor_t *motor, uint8_t id, FDCAN_HandleTypeDef *hcan);
uint8_t cg_motor_enable(CyberGear_Motor_t *motor);
uint8_t cg_motor_stop(CyberGear_Motor_t *motor);
uint8_t cg_motor_set_run_mode(CyberGear_Motor_t *motor, CyberGear_RunMode_t mode);
uint8_t cg_motor_mit_control(CyberGear_Motor_t *motor, const CyberGear_MITCmd_t *cmd);
```

### MIT 原始控制
```c
CyberGear_MITCmd_t cmd = {
    .position = 0.0f,   // 目标位置 (rad), [-4π, +4π]
    .velocity = 1.0f,   // 目标速度 (rad/s), [-30, +30]
    .torque   = 0.5f,   // 前馈力矩 (Nm), [-12, +12]
    .kp = 50.0f, .kd = 2.0f
};
cg_motor_mit_control(&motor, &cmd);
```

---

## 四、高级控制层 (cybergear_control.c) — 本次新增

### 架构

```
应用层状态机
  │
  ├─ cg_ctrl_set_impedance() / set_position_pid() / set_velocity() / set_torque()
  ├─ cg_ctrl_set_target()        设定目标值
  ├─ cg_ctrl_start_trajectory()  启动轨迹
  └─ cg_ctrl_update()          每个控制周期调用 (1kHz) → 自动下发 CAN 帧
       │
       ├─ 计算控制量 (阻抗/PID/轨迹插值)
       ├─ 叠加重力补偿 τ_g = m·g·L·sin(θ)
       ├─ 叠加摩擦补偿 τ_f = τ_c·sign(ω) + b·ω
       └─ cg_motor_mit_control()  下发 MIT 帧
```

### 控制模式

| 模式 | 枚举 | 公式 | 适用场景 |
|------|------|------|---------|
| 阻抗控制 | `CG_CTRL_MODE_IMPEDANCE` | $\tau = K(\theta_d-\theta) + D(\omega_d-\omega) + \tau_{ff}$ | 柔顺着陆、顺应地形 |
| 位置 PID | `CG_CTRL_MODE_POSITION` | $\tau = K_p e + K_i\int e dt + K_d \dot{e}$ | 精确关节定位 |
| 速度控制 | `CG_CTRL_MODE_VELOCITY` | $\tau = K_d(\omega_d-\omega) + \tau_{ff}$ | 转轮模式 |
| 力矩控制 | `CG_CTRL_MODE_TORQUE` | $\tau = \tau_{des}$ | 力控站立、零力矩 |
| 轨迹跟踪 | `CG_CTRL_MODE_TRAJECTORY` | 五次多项式最小 Jerk | 步态足端轨迹 |

### 使用示例

```c
// 速度模式 (轮式): 遥控器摇杆 → 电机
cg_ctrl_set_velocity(&g_ctrl[0], 0.6f);       // Kd=0.6
cg_ctrl_set_target(&g_ctrl[0], 0, 1.0f, 0);   // target_velocity=1.0 rad/s
cg_ctrl_enable(&g_ctrl[0]);
// TIM6 ISR 中: cg_ctrl_update_fixed(&g_ctrl[0], 0.002f);  // 500Hz
```

### 注意事项
- `cg_ctrl_update_fixed()` 在 TIM6 ISR 中以 500Hz 调用
- `online` 仅用于状态监控, 不阻塞 MIT 帧发送
- MIT 帧由 TIM6 ISR 独立发送, 不依赖主循环频率
- 重力/摩擦补偿在 `cg_ctrl_update()` 内部自动叠加到前馈力矩
- 模式切换时自动清零积分项，避免积分冲击
- 所有控制输出自动限幅到 MIT 协议允许范围
