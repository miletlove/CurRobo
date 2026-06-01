# Robo — 系统初始化管线

统一封装所有外设模块的初始化顺序，确保依赖关系正确。

## 文件

| 文件 | 职责 |
|------|------|
| `pipeline.h` | `pipeline_init()` 声明 |
| `pipeline.c` | 初始化管线实现 |

## 初始化顺序

```
pipeline_init()
  ├─ 1. USART1 调试串口    (最早启用, 后续日志可输出)
  ├─ 2. 遥控器 DBUS        (依赖 USART1 DMA)
  ├─ 3. CAN 收发器供电      (POWER_OUT1/2)
  ├─ 4. BMI088 IMU         (独立 SPI2)
  ├─ 5. CAN 总线           (滤波器 → FDCAN_Start → 中断使能)
  ├─ 6. 电机初始化          (绑定 FDCAN1 → 控制节点 → MIT 模式)
  └─ 7. TIM6 1kHz          (最后启动, 确保所有模块就绪)
```

## 使用

```c
// main.c
#include "pipeline.h"

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    // ... 其他 MX_*_Init() ...

    pipeline_init();  // 一行完成所有用户模块初始化

    while (1) {
        data_update_execute();
        // 应用逻辑 ...
    }
}
```

## 与 data_update 的关系

- `pipeline_init()` 负责 **初始化顺序**
- `data_update_*()` 负责 **运行时周期调度**
- 两者解耦：pipeline 是一次性启动，data_update 是持续运行
