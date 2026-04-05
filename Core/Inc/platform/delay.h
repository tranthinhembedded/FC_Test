#ifndef __DELAY_H__
#define __DELAY_H__

#include "main.h"

void Delay_Init(void);
void Delay_us(uint32_t us);
void Delay_ms_blocking(uint32_t ms);

#endif /* __DELAY_H__ */
