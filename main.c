#include "stm32h7xx_hal.h"
#include "runtime.h"

void main(void)
{
    volatile uint32_t i = 0;

    HAL_Init();
    Runtime_Config(RUNTIME_TICK_FRQ_20K);
    Runtime_Start();

    while (1)
    {
        i++;
    }
}
