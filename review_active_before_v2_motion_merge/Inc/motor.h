#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_PWM_MAX       1000

extern int8_t invert_motor_a;
extern int8_t invert_motor_b;

/* Khởi tạo phần cứng động cơ và encoder */
void Motor_Init(void);
void Encoder_Init(void);

/* Điều khiển tốc độ và chiều quay (-1000 đến +1000) */
void Motor_SetSpeed_A(int16_t speed);
void Motor_SetSpeed_B(int16_t speed);
void Motor_SetSpeed(int16_t speed_A, int16_t speed_B);
void Motor_Stop(void);

/* Đảo cực tính động cơ theo thời gian thực */
void Motor_ToggleInvertA(void);
void Motor_ToggleInvertB(void);

/* Đọc số xung Encoder */
int16_t Encoder_GetCount1(void);
int16_t Encoder_GetCount2(void);
void Encoder_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
