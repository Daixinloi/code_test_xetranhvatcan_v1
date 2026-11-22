#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
typedef struct { int unused; } I2C_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
#define I2C_MEMADD_SIZE_8BIT 1
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *, uint16_t, uint16_t,
    uint16_t, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *, uint16_t, uint16_t,
    uint16_t, uint8_t *, uint16_t, uint32_t);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
#endif
