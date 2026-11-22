#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_I2C_ADDR_7BIT       0x68
#define MPU6050_GYRO_LSB_PER_DPS   32.8f /* +/-1000 deg/s */
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

/* Visible in debugger and in Bluetooth diagnostics. Addresses are 7-bit. */
typedef struct {
    uint32_t stage; /* 1=probe, 2=identity, 3=config, 4=ready, 5=read error */
    uint32_t address;
    uint32_t who_am_i;
    uint32_t hal_error;
    uint32_t ack_mask; /* bit0=0x68, bit1=0x69 */
    uint32_t read_failures;
    uint32_t last_read_error;
    uint32_t last_read_error_tick;
} MPU6050_Diagnostics;
extern volatile MPU6050_Diagnostics mpu_diag;

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */

