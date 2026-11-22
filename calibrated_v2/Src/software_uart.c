#include "software_uart.h"

/*
 * Cấu hình TIM2 làm bộ đếm chuẩn:
 * Với xung HSI 8MHz: Prescaler = 7 -> Tần số đếm = 8MHz / 8 = 1MHz (1 tick = đúng 1.0 us).
 */
static void Timer_Delay_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    
    uint32_t timer_clk = HAL_RCC_GetPCLK1Freq();
    RCC_ClkInitTypeDef clk_init;
    uint32_t flash_latency;
    HAL_RCC_GetClockConfig(&clk_init, &flash_latency);
    if (clk_init.APB1CLKDivider != RCC_HCLK_DIV1)
    {
        timer_clk *= 2;
    }

    TIM2->PSC = (timer_clk / 1000000) - 1;
    TIM2->ARR = 0xFFFF;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 |= TIM_CR1_CEN;
}

#define TX_HIGH()   (SOFT_UART_TX_PORT->BSRR = SOFT_UART_TX_PIN)
#define TX_LOW()    (SOFT_UART_TX_PORT->BRR  = SOFT_UART_TX_PIN)
#define RX_READ()   ((SOFT_UART_RX_PORT->IDR & SOFT_UART_RX_PIN) != 0)

/* 1 bit ở 9600 baud = ~104 microsecond */
#define BIT_PERIOD_US   (1000000 / SOFT_UART_BAUDRATE)

/* Hàng đợi đệm vòng (Ring Buffer) nhận dữ liệu từ ngắt EXTI */
#define RX_BUFFER_SIZE  64
static volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;

void SoftUART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    Timer_Delay_Init();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* 1. Cấu hình TX (PA12): Output Push-Pull, mặc định mức HIGH */
    TX_HIGH();
    GPIO_InitStruct.Pin = SOFT_UART_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SOFT_UART_TX_PORT, &GPIO_InitStruct);

    /* 2. Cấu hình RX (PA11): Ngắt ngoài sườn xuống (Falling Edge - Start bit của UART) */
    GPIO_InitStruct.Pin = SOFT_UART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SOFT_UART_RX_PORT, &GPIO_InitStruct);

    /* Cài đặt ưu tiên ngắt cao nhất để không bao giờ bị trễ khi có lệnh tới */
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void SoftUART_TransmitByte(uint8_t data)
{
    uint16_t target = (uint16_t)TIM2->CNT;

    /* 1. Start bit: Kéo mức LOW */
    TX_LOW();
    target += BIT_PERIOD_US;
    while ((uint16_t)(TIM2->CNT - target) > 0x8000);

    /* 2. 8 Data bits (truyền LSB trước) */
    for (uint8_t i = 0; i < 8; i++)
    {
        if (data & (1 << i))
        {
            TX_HIGH();
        }
        else
        {
            TX_LOW();
        }
        target += BIT_PERIOD_US;
        while ((uint16_t)(TIM2->CNT - target) > 0x8000);
    }

    /* 3. Stop bit: Kéo mức HIGH */
    TX_HIGH();
    target += (BIT_PERIOD_US + (BIT_PERIOD_US / 2));
    while ((uint16_t)(TIM2->CNT - target) > 0x8000);
}

void SoftUART_TransmitString(const char *str)
{
    while (*str)
    {
        SoftUART_TransmitByte((uint8_t)*str);
        str++;
    }
}

void SoftUART_Transmit(const uint8_t *pData, uint16_t size)
{
    for (uint16_t i = 0; i < size; i++)
    {
        SoftUART_TransmitByte(pData[i]);
    }
}

bool SoftUART_HasData(void)
{
    return (rx_head != rx_tail);
}

bool SoftUART_ReadByte(uint8_t *data)
{
    if (rx_head == rx_tail)
    {
        return false;
    }
    *data = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
    return true;
}

bool SoftUART_ReceiveByte(uint8_t *data, uint32_t timeout_ms)
{
    if (rx_head != rx_tail)
    {
        return SoftUART_ReadByte(data);
    }

    uint32_t tickstart = HAL_GetTick();
    while (rx_head == rx_tail)
    {
        if ((HAL_GetTick() - tickstart) >= timeout_ms)
        {
            return false;
        }
    }
    return SoftUART_ReadByte(data);
}

uint16_t SoftUART_ReceiveString(char *buffer, uint16_t max_len, uint32_t timeout_ms)
{
    uint16_t count = 0;
    uint8_t ch = 0;

    while (count < (max_len - 1))
    {
        if (SoftUART_ReceiveByte(&ch, timeout_ms))
        {
            buffer[count++] = (char)ch;
            if (ch == '\n' || ch == '\r')
            {
                break;
            }
        }
        else
        {
            break;
        }
    }
    buffer[count] = '\0';
    return count;
}

/*
 * Trình phục vụ ngắt EXTI cho chân PA11 (EXTI11):
 * Bắt tức thì Start Bit sườn xuống ngay khi điện thoại gửi bất kỳ lệnh nào.
 */
void EXTI15_10_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(SOFT_UART_RX_PIN) != RESET)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(SOFT_UART_RX_PIN);

        /* Tạm thời tắt ngắt EXTI line 11 trong lúc đọc byte */
        EXTI->IMR &= ~SOFT_UART_RX_PIN;

        /* Chờ đến giữa Start Bit (52 microsecond) */
        uint16_t target = (uint16_t)TIM2->CNT + (BIT_PERIOD_US / 2);
        while ((uint16_t)(TIM2->CNT - target) > 0x8000);

        /* Nếu chân vẫn ở mức LOW: Start Bit hợp lệ */
        if (RX_READ() == 0)
        {
            uint8_t received_byte = 0;

            /* Lấy mẫu 8 data bits */
            for (uint8_t i = 0; i < 8; i++)
            {
                target += BIT_PERIOD_US;
                while ((uint16_t)(TIM2->CNT - target) > 0x8000);

                if (RX_READ())
                {
                    received_byte |= (1 << i);
                }
            }

            /* Chờ Stop Bit */
            target += BIT_PERIOD_US;
            while ((uint16_t)(TIM2->CNT - target) > 0x8000);

            /* Đưa byte vào bộ đệm vòng FIFO */
            uint8_t next_head = (rx_head + 1) % RX_BUFFER_SIZE;
            if (next_head != rx_tail)
            {
                rx_buffer[rx_head] = received_byte;
                rx_head = next_head;
            }
        }

        /* Xóa cờ ngắt tồn đọng và bật lại ngắt EXTI line 11 */
        __HAL_GPIO_EXTI_CLEAR_IT(SOFT_UART_RX_PIN);
        EXTI->IMR |= SOFT_UART_RX_PIN;
    }
}
