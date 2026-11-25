#include "bsp_dwt.h"
#include "stm32f4xx.h"

void BSP_DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t BSP_DWT_GetUs(void)
{
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

void BSP_DWT_DelayUs(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks)
    {
        /* 精确硬件周期等待，供后续 DS18B20 与耗时打点使用 */
    }
}
