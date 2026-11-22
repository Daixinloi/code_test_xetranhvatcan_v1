#ifndef __SOFTWARE_UART_H
#define __SOFTWARE_UART_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Cấu hình chân Software UART:
 * - STM32 TX: PA12 (kết nối chân RX của HC-05)
 * - STM32 RX: PA11 (kết nối chân TX của HC-05, dùng ngắt EXTI11 để bắt lệnh tức thì 100%)
 * - Baudrate: 9600 bps
 */
#define SOFT_UART_TX_PORT       GPIOA
#define SOFT_UART_TX_PIN        GPIO_PIN_12

#define SOFT_UART_RX_PORT       GPIOA
#define SOFT_UART_RX_PIN        GPIO_PIN_11

#define SOFT_UART_BAUDRATE      9600

void     SoftUART_Init(void);
void     SoftUART_TransmitByte(uint8_t data);
void     SoftUART_TransmitString(const char *str);
void     SoftUART_Transmit(const uint8_t *pData, uint16_t size);
bool     SoftUART_HasData(void);
bool     SoftUART_ReadByte(uint8_t *data);
bool     SoftUART_ReceiveByte(uint8_t *data, uint32_t timeout_ms);
uint16_t SoftUART_ReceiveString(char *buffer, uint16_t max_len, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __SOFTWARE_UART_H */
