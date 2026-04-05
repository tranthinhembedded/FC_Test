#include "platform/delay.h"
#include "platform/tim.h"

static uint8_t delay_initialized = 0U;

void Delay_Init(void)
{
    if (delay_initialized == 0U) {
        HAL_TIM_Base_Start(&htim2);
        delay_initialized = 1U;
    }
}

void Delay_us(uint32_t us)
{
    
    uint32_t start;

    Delay_Init();
    start = TIM2->CNT;
    while ((uint32_t)(TIM2->CNT - start) < us) {
    }
}

void Delay_ms_blocking(uint32_t ms)
{
    while (ms-- > 0U) {
        Delay_us(1000U);
    }
}
