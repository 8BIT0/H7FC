#include "reboot.h"
#include "shell_port.h"
#include "kernel.h"

void ReBoot(void)
{
    Kernel_reboot();
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, ReBoot, ReBoot, System ReBoot);
