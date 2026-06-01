# Modules — CyberGear 电机驱动库

本模块包含两层 API：

| 文件 | 层级 | 职责 |
|------|------|------|
| `cybergear_motor.h/.c` | **驱动层** | CAN 帧收发、MIT 模式组包、反馈解析 |
| `cybergear_control.h/.c` | **高级控制层** | 阻抗控制、PID、轨迹跟踪、重力/摩擦补偿 |

---

## 一、硬件依赖
- FDCAN1 / FDCAN2, 扩展帧 29-bit ID

## 二、CubeMX 要求
- FDCAN: Classic CAN, 1Mbps, Extended ID
- CAN 收发器供电: POWER_OUT1/POWER_OUT2
- 电机 ID 通过 CyberGear 上位机预先配置

---

## 三、驱动层 (cybergear_motor.c)

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

### 典型四足机器人参数参考

| 参数 | 摆动相 | 站立相 |
|------|--------|--------|
| 刚度 K (Nm/rad) | 10 ~ 50 | 100 ~ 300 |
| 阻尼 D (Nm/(rad/s)) | 0.5 ~ 3 | 3 ~ 10 |
| 重力补偿 m (kg) | 0.5 ~ 2.0 | — |
| 库伦摩擦 (Nm) | 0.05 ~ 0.3 | — |

### 使用示例

```c
// 1. 定义电机和控制节点数组
CyberGear_Motor_t    g_motors[8];
CyberGear_CtrlNode_t g_ctrl[8];

// 2. 初始化
for (int i = 0; i < 8; i++) {
    cg_motor_init(&g_motors[i], i + 1, (i < 4) ? &hfdcan1 : &hfdcan2);
    cg_ctrl_init(&g_ctrl[i], &g_motors[i]);
}

// 3. 设置控制模式 (以髋关节阻抗控制为例)
cg_ctrl_set_impedance(&g_ctrl[0], 50.0f, 3.0f);   // 刚度 50, 阻尼 3
cg_ctrl_set_gravity_comp(&g_ctrl[0], 1.5f, 0.25f); // 质量 1.5kg, 质心距 0.25m
cg_ctrl_set_friction_comp(&g_ctrl[0], 0.1f, 0.03f); // 库伦 0.1Nm, 粘滞 0.03
cg_ctrl_set_target(&g_ctrl[0], 0.5f, 0.0f, 0.0f);  // 目标角度 0.5 rad

// 4. 使能
cg_ctrl_enable(&g_ctrl[0]);

// 5. 每个控制周期 (1kHz 任务中)
void ctrl_task(void) {
    for (int i = 0; i < 8; i++) {
        cg_ctrl_sync_online(&g_ctrl[i]);  // 同步在线状态
        cg_ctrl_update(&g_ctrl[i]);       // 计算并下发
    }
}
```

### 注意事项
- `cg_ctrl_update()` 建议在 FreeRTOS 任务中以 1kHz 频率调用
- 重力/摩擦补偿在 `cg_ctrl_update()` 内部自动叠加到前馈力矩
- 模式切换时自动清零积分项，避免积分冲击
- 所有控制输出自动限幅到 MIT 协议允许范围
