# App — CyberGear 电机驱动

## 初始化
```c
#include "cybergear_motor.h"
CyberGear_Motor_t motor;
cg_motor_init(&motor, 0x01, &hfdcan1);
cg_motor_enable(&motor);
cg_motor_set_run_mode(&motor, CG_RUN_MIT);
```

## MIT 控制
```c
CyberGear_MITCmd_t cmd = {
    .position=0, .velocity=1.0f, .torque=0.5f, .kp=0, .kd=2.0f
};
cg_motor_mit_control(&motor, &cmd);
```

## 注意
- 需要在 main.c 定义 `CyberGear_Motor_t g_cg_motors[]` 和 `g_cg_motor_count`
- 电机反馈通过 CAN 回调自动更新
