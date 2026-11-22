#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_I2C_ADDR_7BIT       0x68
#define MPU6050_I2C_ADDR_8BIT       (MPU6050_I2C_ADDR_7BIT << 1) // 0xD0

typedef struct {
    int16_t Accel_X;
    int16_t Accel_Y;
    int16_t Accel_Z;
    int16_t Gyro_X;
    int16_t Gyro_Y;
    int16_t Gyro_Z;
    float Temperature;
} MPU6050_Data_t;

bool MPU6050_Init(I2C_HandleTypeDef *hi2c);
bool MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */

