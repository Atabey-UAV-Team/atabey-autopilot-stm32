#include "time.h"
#include "stm32f4xx.h"

/*
 * DWT-based timing.
 * CYCCNT wraps at 2^32 cycles (~50 s @ 84 MHz). 32-bit µs subtraction is
 * still correct across one wrap, so consumers must compute deltas, not
 * compare absolute timestamps across long gaps.
 */

static uint32_t s_cycles_per_us = 1U;

void time_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;

    s_cycles_per_us = SystemCoreClock / 1000000U;
    if (s_cycles_per_us == 0U) {
        s_cycles_per_us = 1U;
    }
}

uint32_t micros(void)
{
    return DWT->CYCCNT / s_cycles_per_us;
}

uint32_t millis(void)
{
    return micros() / 1000U;
}
