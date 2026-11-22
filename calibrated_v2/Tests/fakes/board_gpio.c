#include "stm32f1xx_hal.h"
GPIO_TypeDef fake_gpiob;
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState value)
{
    if (value == GPIO_PIN_SET) port->ODR |= pin;
    else port->ODR &= ~(uint32_t)pin;
}
