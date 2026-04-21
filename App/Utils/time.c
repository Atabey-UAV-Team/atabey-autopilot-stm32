#include "time.h"
#include "stm32f4xx.h"

/*
 * DWT_CYCCNT wraps at 2^32 cycles (~42 seconds at 168 MHz).
 * micros() is accurate as long as callers run more often than that.
 */

void time_init(void)
{
    /* Enable DWT unit and reset cycle counter */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;

    cycles_per_us = SystemCoreClock / 1000000U;
}

uint32_t micros(void)
{
    /* SystemCoreClock is set by CubeMX startup; no HAL calls needed here */
    return DWT->CYCCNT / cycles_per_us;
}
