/**
 * @file    bsp_imu.c
 * @brief   BMI088 IMU BSP 封装
 * @note    基于 CtrBoard-H7_IMU 例程移植
 *          温控功能暂未启用 (后续需要时取消注释即可)
 */
#include "bsp_imu.h"
#include "BMI088driver.h"

uint8_t bsp_imu_init(void)
{
    return BMI088_init();
}

void bsp_imu_read(float gyro[3], float accel[3])
{
    float temp; /* 占位, 后续启用温控时使用 */
    BMI088_read(gyro, accel, &temp);
}
