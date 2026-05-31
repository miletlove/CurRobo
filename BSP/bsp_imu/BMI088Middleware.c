/**
 * @file    BMI088Middleware.c
 * @brief   BMI088 中间层 — SPI 通信 + CS 控制 + 延时
 * @note    SPI2: PC1(MOSI)/PC2(MISO)/PB13(SCK)
 *          CS_ACC: PC0, CS_GYRO: PC3
 */
#include "BMI088Middleware.h"
#include "main.h"
#include "spi.h"

/* SPI 单元绑定 */
#define BMI088_USING_SPI_UNIT   hspi2
extern SPI_HandleTypeDef BMI088_USING_SPI_UNIT;

void BMI088_GPIO_init(void) {}
void BMI088_com_init(void) {}

void BMI088_delay_ms(uint16_t ms)
{
    while (ms--) BMI088_delay_us(1000);
}

void BMI088_delay_us(uint16_t us)
{
    uint32_t ticks = us * 480;
    uint32_t told = SysTick->VAL, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD;
    while (1) {
        tnow = SysTick->VAL;
        if (tnow != told) {
            tcnt += (tnow < told) ? (told - tnow) : (reload - tnow + told);
            told = tnow;
            if (tcnt >= ticks) break;
        }
    }
}

void BMI088_ACCEL_NS_L(void) { HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET); }
void BMI088_ACCEL_NS_H(void) { HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET); }
void BMI088_GYRO_NS_L(void)  { HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET); }
void BMI088_GYRO_NS_H(void)  { HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET); }

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&BMI088_USING_SPI_UNIT, &txdata, &rx_data, 1, 1000);
    return rx_data;
}
