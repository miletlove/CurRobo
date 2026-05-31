# App — CyberGear 电机驱动 (MIT 模式)

## 硬件
- FDCAN1 / FDCAN2, 扩展帧 29-bit ID

## 数据结构
```c
CyberGear_Motor_t motor;
cg_motor_init(&motor, motor_id, &hfdcan1);  // motor_id: 0x01~0x7F
```

## CubeMX 要求
- FDCAN: Classic CAN, 1Mbps, Extended ID
- CAN 收发器供电: POWER_OUT1/POWER_OUT2
- 电机 ID 通过 CyberGear 上位机预先配置

## API
```c
void    cg_motor_init(CyberGear_Motor_t *motor, uint8_t id, FDCAN_HandleTypeDef *hcan);
uint8_t cg_motor_enable(CyberGear_Motor_t *motor);
uint8_t cg_motor_stop(CyberGear_Motor_t *motor);
uint8_t cg_motor_set_run_mode(CyberGear_Motor_t *motor, CyberGear_RunMode_t mode);
uint8_t cg_motor_mit_control(CyberGear_Motor_t *motor, const CyberGear_MITCmd_t *cmd);
```

## MIT 控制
```c
CyberGear_MITCmd_t cmd = {
    .position = 0.0f,   // 目标位置 (rad), [-4, +4]
    .velocity = 1.0f,   // 目标速度 (rad/s), [-30, +30]
    .torque   = 0.5f,   // 前馈力矩 (Nm), [-12, +12]
    .kp = 0.0f, .kd = 2.0f
};
cg_motor_mit_control(&motor, &cmd);
```

## 注意事项
- 使能前必须设置运行模式 (CG_RUN_MIT)
- 电机反馈通过 `can1_rx_callback` / `can2_rx_callback` 自动更新 `motor.feedback`
- 需要在 main.c 定义全局数组 `g_cg_motors[]` 和 `g_cg_motor_count`
