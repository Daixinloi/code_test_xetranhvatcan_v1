#include "motor.h"

/* Đã chỉnh: invert_motor_a = 1, invert_motor_b = 0 để bấm TIẾN xe chạy TIẾN, bấm LÙI xe chạy LÙI */
int8_t invert_motor_a = 1;
int8_t invert_motor_b = 0;

void Motor_ToggleInvertA(void)
{
    invert_motor_a = !invert_motor_a;
}

void Motor_ToggleInvertB(void)
{
    invert_motor_b = !invert_motor_b;
}

void Motor_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Cấu hình chân điều khiển hướng: PA0 (AIN1), PA1 (AIN2), PA2 (BIN1), PA3 (BIN2) */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Cấu hình chân PWM: PA8 (TIM1_CH1) và PA9 (TIM1_CH2) */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Cấu hình TIM1 PWM ~1 kHz (Clock = 8 MHz) */
    TIM1->PSC = (SystemCoreClock / 1000000) - 1; // 1 MHz counter clock
    TIM1->ARR = MOTOR_PWM_MAX - 1;               // 1000 counts => 1 kHz PWM
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;

    TIM1->CCMR1 = (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE |
                  (6 << TIM_CCMR1_OC2M_Pos) | TIM_CCMR1_OC2PE;

    TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    TIM1->BDTR = TIM_BDTR_MOE;
    TIM1->EGR  = TIM_EGR_UG;
    TIM1->CR1  = TIM_CR1_CEN | TIM_CR1_ARPE;
}

void Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Encoder 1: PA6 (TIM3_CH1), PA7 (TIM3_CH2) */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    TIM3->SMCR = (3 << TIM_SMCR_SMS_Pos);
    TIM3->CCMR1 = (1 << TIM_CCMR1_CC1S_Pos) | (1 << TIM_CCMR1_CC2S_Pos);
    TIM3->CCER = 0;
    TIM3->ARR = 0xFFFF;
    TIM3->CNT = 0;
    TIM3->CR1 = TIM_CR1_CEN;

    /* Encoder 2: PB6 (TIM4_CH1), PB7 (TIM4_CH2) */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    TIM4->SMCR = (3 << TIM_SMCR_SMS_Pos);
    TIM4->CCMR1 = (1 << TIM_CCMR1_CC1S_Pos) | (1 << TIM_CCMR1_CC2S_Pos);
    TIM4->CCER = 0;
    TIM4->ARR = 0xFFFF;
    TIM4->CNT = 0;
    TIM4->CR1 = TIM_CR1_CEN;
}

void Motor_SetSpeed_A(int16_t speed)
{
    if (invert_motor_a)
    {
        speed = -speed;
    }

    if (speed > MOTOR_PWM_MAX) speed = MOTOR_PWM_MAX;
    if (speed < -MOTOR_PWM_MAX) speed = -MOTOR_PWM_MAX;

    if (speed > 0)
    {
        /* Quay thuận: AIN1 = 1, AIN2 = 0 */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
        TIM1->CCR1 = (uint32_t)speed;
    }
    else if (speed < 0)
    {
        /* Quay ngược: AIN1 = 0, AIN2 = 1 */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
        TIM1->CCR1 = (uint32_t)(-speed);
    }
    else
    {
        /* Dừng / Phanh */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
        TIM1->CCR1 = 0;
    }
}

void Motor_SetSpeed_B(int16_t speed)
{
    if (invert_motor_b)
    {
        speed = -speed;
    }

    if (speed > MOTOR_PWM_MAX) speed = MOTOR_PWM_MAX;
    if (speed < -MOTOR_PWM_MAX) speed = -MOTOR_PWM_MAX;

    if (speed > 0)
    {
        /* Quay thuận: BIN1 = 1, BIN2 = 0 */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        TIM1->CCR2 = (uint32_t)speed;
    }
    else if (speed < 0)
    {
        /* Quay ngược: BIN1 = 0, BIN2 = 1 */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
        TIM1->CCR2 = (uint32_t)(-speed);
    }
    else
    {
        /* Dừng / Phanh */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        TIM1->CCR2 = 0;
    }
}

void Motor_SetSpeed(int16_t speed_A, int16_t speed_B)
{
    Motor_SetSpeed_A(speed_A);
    Motor_SetSpeed_B(speed_B);
}

void Motor_Stop(void)
{
    Motor_SetSpeed(0, 0);
}

int16_t Encoder_GetCount1(void)
{
    return (int16_t)TIM3->CNT;
}

int16_t Encoder_GetCount2(void)
{
    return (int16_t)TIM4->CNT;
}

void Encoder_Reset(void)
{
    TIM3->CNT = 0;
    TIM4->CNT = 0;
}
