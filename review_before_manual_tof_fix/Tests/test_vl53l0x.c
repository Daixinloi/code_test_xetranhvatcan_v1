#include "vl53l0x.h"
#include <stdio.h>
#include <stdlib.h>
#define CHECK(c) do { if (!(c)) { fprintf(stderr,"FAIL %d: %s\n",__LINE__,#c); exit(1); } } while (0)
static unsigned failure, clear_attempts;
static bool ready = true;
static uint16_t raw = 123;
static I2C_HandleTypeDef bus;
uint32_t HAL_GetTick(void) { return 0; }
void HAL_Delay(uint32_t ms) { (void)ms; }
uint32_t HAL_I2C_GetError(I2C_HandleTypeDef *b) { (void)b; return 0x220; }
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *b, uint16_t a, uint16_t r,
    uint16_t rs, uint8_t *d, uint16_t n, uint32_t t)
{
    (void)b; (void)a; (void)rs; (void)t;
    if (r == 0x13) {
        if (failure == 1) return HAL_ERROR;
        *d = ready ? 1 : 0;
    } else if (r == 0x1E && n == 2) {
        if (failure == 2) return HAL_ERROR;
        d[0] = raw >> 8; d[1] = raw & 255;
    }
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *b, uint16_t a, uint16_t r,
    uint16_t rs, uint8_t *d, uint16_t n, uint32_t t)
{
    (void)b; (void)a; (void)rs; (void)d; (void)n; (void)t;
    if (r == 0x0B) {
        clear_attempts++;
        if (failure == 3) return HAL_ERROR;
    }
    return HAL_OK;
}
int main(void)
{
    VL53L0X_Dev dev = {.hi2c = &bus, .address = 0x64, .timeout = 20};
    ready = false;
    CHECK(VL53L0X_ReadRangeContinuousMillimeters(&dev) == 0 && dev.last_error == 0);
    CHECK(clear_attempts == 0);
    ready = true;
    for (unsigned fault = 1; fault <= 3; ++fault) {
        failure = fault; clear_attempts = 0;
        CHECK(VL53L0X_ReadRangeContinuousMillimeters(&dev) == 0);
        CHECK(dev.last_error == 0x220);
        CHECK(clear_attempts == (fault == 3 ? 1u : 0u));
        failure = 0;
        CHECK(VL53L0X_ReadRangeContinuousMillimeters(&dev) == raw);
        CHECK(dev.last_error == 0);
    }
    raw = 8191;
    CHECK(VL53L0X_ReadRangeContinuousMillimeters(&dev) == 8191 && dev.last_error == 0);
    puts("PASS: I2C errors preserved, partial samples rejected, no premature interrupt clear, recovery read");
    return 0;
}