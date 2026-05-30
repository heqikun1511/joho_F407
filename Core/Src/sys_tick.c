#include "sys_tick.h"

static volatile uint32_t countdown_end = 0;

void SysTick_DelayMs(uint32_t ms)
{
    HAL_Delay(ms);
}

void SysTick_CountdownBegin(uint32_t ms)
{
    countdown_end = HAL_GetTick() + ms;
}

uint8_t SysTick_CountdownIsTimeout(void)
{
    return (HAL_GetTick() >= countdown_end) ? 1 : 0;
}

void SysTick_CountdownCancel(void)
{
    countdown_end = 0;
}
