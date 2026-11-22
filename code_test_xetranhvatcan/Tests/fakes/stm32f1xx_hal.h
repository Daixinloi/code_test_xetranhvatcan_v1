#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
typedef struct { int unused; } I2C_HandleTypeDef;
typedef struct { uint32_t ODR; } GPIO_TypeDef;
extern GPIO_TypeDef fake_gpiob;
#define GPIOB (&fake_gpiob)
#define GPIO_PIN_8 (1u << 8)
#define GPIO_PIN_9 (1u << 9)
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
#define I2C_MEMADD_SIZE_8BIT 1
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *, uint16_t, uint16_t,
    uint16_t, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *, uint16_t, uint16_t,
    uint16_t, uint8_t *, uint16_t, uint32_t);
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
#endif
