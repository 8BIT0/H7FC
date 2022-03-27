#include "reboot.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"

void ReBoot(void)
{
    __set_FAULTMASK(1);

    NVIC_SystemReset();
}
