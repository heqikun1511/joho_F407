#ifndef __SYS_TICK_H
#define __SYS_TICK_H

#include "stm32f4xx.h"

void SysTick_DelayMs(uint32_t ms);
void SysTick_CountdownBegin(uint32_t ms);
uint8_t SysTick_CountdownIsTimeout(void);
void SysTick_CountdownCancel(void);

#endif
