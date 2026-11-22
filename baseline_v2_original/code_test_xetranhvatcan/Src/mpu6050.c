#include "mpu6050.h"

#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75

bool MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t check = 0;
    uint8_t data = 0;

    /* 1. Kiểm tra mã WHO_AM_I (mặc định là 0x68) */
    if (HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDR_8BIT, REG_WHO_AM_I, 1, &check, 1, 100) != HAL_OK)
    {
        return false;
    }

    if (check != 0x68)
    {
        return false;
    }

    /* 2. Đánh thức MPU-6050 (Reset bit SLEEP trong thanh ghi PWR_MGMT_1) */
    data = 0x00;
    if (HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR_8BIT, REG_PWR_MGMT_1, 1, &data, 1, 100) != HAL_OK)
    {
        return false;
    }

    /* 3. Đặt Sample Rate Divider = 0 (tần số lấy mẫu = 1kHz) */
    data = 0x07;
    HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR_8BIT, REG_SMPLRT_DIV, 1, &data, 1, 100);

    /* 4. Cấu hình Accel Range: ±2g */
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR_8BIT, REG_ACCEL_CONFIG, 1, &data, 1, 100);

    /* 5. Cấu hình Gyro Range: ±250 deg/s */
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR_8BIT, REG_GYRO_CONFIG, 1, &data, 1, 100);

    return true;
}

bool MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data)
{
    uint8_t raw_buf[14];

    /* Đọc liên tục 14 bytes từ thanh ghi 0x3B (ACCEL_XOUT_H) */
    if (HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDR_8BIT, REG_ACCEL_XOUT_H, 1, raw_buf, 14, 100) != HAL_OK)
    {
        return false;
    }

    data->Accel_X = (int16_t)((raw_buf[0] << 8) | raw_buf[1]);
    data->Accel_Y = (int16_t)((raw_buf[2] << 8) | raw_buf[3]);
    data->Accel_Z = (int16_t)((raw_buf[4] << 8) | raw_buf[5]);

    int16_t temp_raw = (int16_t)((raw_buf[6] << 8) | raw_buf[7]);
    data->Temperature = ((float)temp_raw / 340.0f) + 36.53f;

    data->Gyro_X = (int16_t)((raw_buf[8] << 8) | raw_buf[9]);
    data->Gyro_Y = (int16_t)((raw_buf[10] << 8) | raw_buf[11]);
    data->Gyro_Z = (int16_t)((raw_buf[12] << 8) | raw_buf[13]);

    return true;
}

