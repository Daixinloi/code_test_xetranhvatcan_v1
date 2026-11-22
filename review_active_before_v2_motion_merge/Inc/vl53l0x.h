#ifndef __VL53L0X_H
#define __VL53L0X_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VL53L0X_DEFAULT_ADDR_7BIT     0x29
#define VL53L0X_DEFAULT_ADDR_8BIT     (VL53L0X_DEFAULT_ADDR_7BIT << 1) // 0x52

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address;            // Địa chỉ 8-bit giao tiếp HAL
    uint8_t stop_variable;
    uint32_t timeout;           // Timeout tính bằng ms
    uint16_t raw_mm;
    uint8_t device_status;
    uint8_t reference_spads;
    uint32_t samples;
    bool io_failed;
} VL53L0X_Dev;

/* Các hàm API cho cảm biến VL53L0X */
bool     VL53L0X_Init(VL53L0X_Dev *dev, I2C_HandleTypeDef *hi2c, uint8_t address_8bit);
bool     VL53L0X_SetAddress(VL53L0X_Dev *dev, uint8_t new_address_7bit);
bool     VL53L0X_StartContinuous(VL53L0X_Dev *dev);
void     VL53L0X_StopContinuous(VL53L0X_Dev *dev);
uint16_t VL53L0X_ReadRangeContinuousMillimeters(VL53L0X_Dev *dev);
bool     VL53L0X_ReadMeasurement(VL53L0X_Dev *dev, uint16_t *millimeters);
uint16_t VL53L0X_ReadRangeSingleMillimeters(VL53L0X_Dev *dev);

#ifdef __cplusplus
}
#endif

#endif /* __VL53L0X_H */
