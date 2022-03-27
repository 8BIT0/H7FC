#include "reboot.h"
#include "shell.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"

void ReBoot(void)
{
    __set_FAULTMASK(1);

    NVIC_SystemReset();
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, ReBoot, ReBoot, System ReBoot);
