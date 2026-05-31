# BSP — BMI088 IMU (6 轴惯性传感器)

## 硬件
- **SPI2**: PC1(MOSI) / PC2(MISO) / PB13(SCK)
- **CS_ACC**: PC0, **CS_GYRO**: PC3
- **INT_ACC**: PE10, **INT_GYRO**: PE12

## CubeMX 配置
| 参数 | 值 |
|------|-----|
| SPI2 Mode | Master, 2 Lines, 8-bit |
| CPOL/CPHA | HIGH / 2EDGE |
| Baud Rate | Prescaler 32 |
| NSS | Software |
| DMA | (可选) SPI2_RX/TX |

## API
```c
uint8_t bsp_imu_init(void);                    // 初始化, 返回0=成功
void    bsp_imu_read(float gyro[3], float accel[3]);  // 读取数据
```

## 初始化
```c
#include "bsp_imu.h"
if (bsp_imu_init() != 0) Error_Handler();
```

## 温控功能
当前版本**未启用**温度读取。BMI088 自带温度传感器，如需启用：

1. 取消 `bsp_imu.h` 中 `bsp_imu_read_temp()` 的注释
2. `bsp_imu.c` 中传入 `temp` 指针即可
3. 参考 `CtrBoard-H7_IMU_TempCtrl` 例程了解温控 PID 实现

## 文件结构
```
BSP/bsp_imu/
├── bsp_imu.h/c           ← BSP 封装
├── BMI088driver.h/c      ← 核心驱动 (init/read)
├── BMI088Middleware.h/c  ← SPI/GPIO 中间层
└── BMI088reg.h           ← 寄存器定义
```
