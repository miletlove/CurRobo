# App/task — 应用任务调度

遥控器→电机控制工作流: 初始化 → 安全保护 → 执行。

## 调用链路

```
main()
  → pipeline_init()       // BSP 层: CAN/IMU/电机/TIM6
  → app_task_init()       // App 层: wheel_init() 配置速度模式+使能
  → while(1) {
      app_task_run()      // data_update_execute() + wheel_update()
    }
```

## 安全保护

| 条件 | 行为 |
|------|------|
| 遥控器离线 | 目标速度置零 |
| 电机离线 | 目标速度置零 |
| 电机未使能 | 目标速度置零 |

## API

```c
void app_task_init(void);          // 任务初始化 (main 中调用一次)
void app_task_run(void);           // 任务执行 (while 循环中调用)
void app_task_wheel_update(void);  // 轮式控制更新 (含安全保护)
```

## 依赖

- `Robo/wheel/` — 轮式控制算法
- `App/data_update/` — TIM6 调度
- `Modules/cybergear/` — 电机控制
- `Modules/remote_control/` — 遥控器数据
