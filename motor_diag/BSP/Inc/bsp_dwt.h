#ifndef __BSP_DWT_H
#define __BSP_DWT_H

#include <stdint.h>

void     BSP_DWT_Init(void);
uint32_t BSP_DWT_GetUs(void);
void     BSP_DWT_DelayUs(uint32_t us);

#endif
