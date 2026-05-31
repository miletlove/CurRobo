/**
 * @file    bsp_imu.h
 * @brief   BMI088 IMU BSP 接口
 */
#ifndef BSP_IMU_H
#define BSP_IMU_H

#include "struct_typedef.h"

/**
 * @brief  初始化 BMI088 (SPI2 + 寄存器配置)
 * @retval 0=成功, 非0=失败 (参见 BMI088driver.h 错误码)
 */
uint8_t bsp_imu_init(void);

/**
 * @brief  读取 BMI088 数据
 * @param  gyro   [输出] 陀螺仪 xyz (rad/s)
 * @param  accel  [输出] 加速度 xyz (m/s²)
 * @note   温度读取暂未启用 (后续需要时取消注释即可)
 */
void bsp_imu_read(float gyro[3], float accel[3]);

/* TODO: 温控相关 — 后续加入时取消下面注释
 * void bsp_imu_read_temp(float *temp);
 */

#endif
