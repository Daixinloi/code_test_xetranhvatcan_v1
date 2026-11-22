#include "mpu6050.h"

#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75

volatile MPU6050_Diagnostics mpu_diag;
static uint16_t active_address = MPU6050_I2C_ADDR_8BIT;

static bool WriteRegister(I2C_HandleTypeDef *bus, uint8_t reg, uint8_t value)
{
    if (HAL_I2C_Mem_Write(bus, active_address, reg, 1, &value, 1, 20) != HAL_OK) {
        mpu_diag.hal_error = HAL_I2C_GetError(bus);
        return false;
    }
    uint8_t actual;
    if (HAL_I2C_Mem_Read(bus, active_address, reg, 1, &actual, 1, 20) != HAL_OK) {
        mpu_diag.hal_error = HAL_I2C_GetError(bus);
        return false;
    }
    return actual == value;
}

bool MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    mpu_diag.stage = 1;
    mpu_diag.address = 0;
    mpu_diag.who_am_i = 0;
    mpu_diag.hal_error = 0;
    mpu_diag.ack_mask = 0;
    /* AD0 may select either address; WHO_AM_I remains 0x68 at both. */
    for (unsigned i = 0; i < 2; ++i) {
        uint16_t address = (0x68u + i) << 1;
        if (HAL_I2C_IsDeviceReady(hi2c, address, 3, 20) == HAL_OK)
            mpu_diag.ack_mask |= 1u << i;
    }
    for (unsigned i = 0; i < 2; ++i) {
        if (!(mpu_diag.ack_mask & (1u << i))) continue;
        uint8_t identity = 0;
        active_address = (0x68u + i) << 1;
        mpu_diag.address = 0x68u + i;
        mpu_diag.stage = 2;
        if (HAL_I2C_Mem_Read(hi2c, active_address, REG_WHO_AM_I, 1,
                             &identity, 1, 20) != HAL_OK) {
            mpu_diag.hal_error = HAL_I2C_GetError(hi2c);
            continue;
        }
        mpu_diag.who_am_i = identity;
        /* MPU6050=0x68, MPU6500=0x70. Both share the gyro/accel registers
         * used here. Reject other identities rather than silently assuming. */
        if (identity != 0x68 && identity != 0x70) continue;
        mpu_diag.stage = 3;
        if (!WriteRegister(hi2c, REG_PWR_MGMT_1, 0x01)) return false;
        HAL_Delay(100);
        if (!WriteRegister(hi2c, REG_CONFIG, 0x03) ||
            !WriteRegister(hi2c, REG_SMPLRT_DIV, 0x04) ||
            !WriteRegister(hi2c, REG_ACCEL_CONFIG, 0x00) ||
            !WriteRegister(hi2c, REG_GYRO_CONFIG, 0x10)) return false;
        mpu_diag.stage = 4;
        mpu_diag.hal_error = 0;
        return true;
    }
    mpu_diag.hal_error = HAL_I2C_GetError(hi2c);
    return false;
}

bool MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data)
{
    uint8_t raw_buf[14];

    /* Đọc liên tục 14 bytes từ thanh ghi 0x3B (ACCEL_XOUT_H) */
    bool received = false;
    /* One retry handles an isolated NACK without permanently disabling MPU.
     * The bounded wait stays below the 100 ms gyro sampling limit. */
    for (unsigned attempt = 0; attempt < 2; ++attempt) {
        if (HAL_I2C_Mem_Read(hi2c, active_address, REG_ACCEL_XOUT_H, 1,
                             raw_buf, 14, 20) == HAL_OK) {
            received = true;
            break;
        }
        mpu_diag.read_failures++;
        mpu_diag.last_read_error = HAL_I2C_GetError(hi2c);
        mpu_diag.last_read_error_tick = HAL_GetTick();
        if (attempt == 0) HAL_Delay(1);
    }
    if (!received) {
        mpu_diag.stage = 5;
        mpu_diag.hal_error = HAL_I2C_GetError(hi2c);
        return false;
    }
    mpu_diag.stage = 4;
    mpu_diag.hal_error = 0;

    data->Accel_X = (int16_t)((raw_buf[0] << 8) | raw_buf[1]);
    data->Accel_Y = (int16_t)((raw_buf[2] << 8) | raw_buf[3]);
    data->Accel_Z = (int16_t)((raw_buf[4] << 8) | raw_buf[5]);

    int16_t temp_raw = (int16_t)((raw_buf[6] << 8) | raw_buf[7]);
    data->Temperature = ((float)temp_raw / 340.0f) + 36.53f;
    if (mpu_diag.who_am_i == 0x70)
        data->Temperature = (float)temp_raw / 333.87f + 21.0f;

    data->Gyro_X = (int16_t)((raw_buf[8] << 8) | raw_buf[9]);
    data->Gyro_Y = (int16_t)((raw_buf[10] << 8) | raw_buf[11]);
    data->Gyro_Z = (int16_t)((raw_buf[12] << 8) | raw_buf[13]);

    return true;
}

