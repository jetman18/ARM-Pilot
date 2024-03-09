#ifndef _MPU6050_H_
#define _MPU6050_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f1xx_hal.h"
#include "axis.h"
//#define SPI
#define I2C

void mpu6050_gyro_get_raw(axis3_t *k);
void mpu6050_acc_get_raw(axis3_t *k);
void mpu_read_all(axis3_t *acc, axis3_t *gyr);
int8_t mpu6050connectFlag();
int8_t mpu6050Connection();
int8_t mpu6050_init
(
#ifdef I2C
    I2C_HandleTypeDef *hi2c
#endif   
);
int16_t get_axis_register(uint8_t addr);
void write_axis_register(uint8_t addr,uint8_t val);
#ifdef __cplusplus
}
#endif

#endif
