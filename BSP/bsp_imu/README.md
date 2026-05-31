# BSP — BMI088 IMU (6 轴惯性传感器)

## 硬件连线
| BMI088 | MCU 引脚 |
|--------|---------|
| SPI2 MOSI | PC1 (AF5) |
| SPI2 MISO | PC2 (AF5) |
| SPI2 SCK | PB13 (AF5) |
| ACC_CS | PC0 |
| GYRO_CS | PC3 |

## CubeMX 配置
| 参数 | 值 |
|------|-----|
| SPI2 Mode | Master, 2 Lines, 8-bit |
| CPOL/CPHA | HIGH / 2EDGE |
| Baud Rate | Prescaler 32 |
| NSS | Software |

## 初始化
```c
#include "bsp_imu.h"
// 在 MX_SPI2_Init() 之后调用
if (bsp_imu_init() != 0) Error_Handler();
```

## 读取数据
```c
float gyro[3], accel[3];
bsp_imu_read(gyro, accel);
// gyro[0/1/2] = 角速度 rad/s, accel[0/1/2] = 加速度 m/s²
```

## 温控功能 (暂未启用)
需要时取消 `bsp_imu.h` 中相关注释，参考 `CtrBoard-H7_IMU_TempCtrl` 例程。
