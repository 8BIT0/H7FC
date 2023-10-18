#include "Srv_OsCommon.h"

/* external function */

SrvOsCommon_TypeDef SrvOsCommon = {
    .get_os_ms = osKernelSysTick,
    .delay_ms = osDelay,
    .precise_delay = osDelayUntil,
    .malloc = pvPortMalloc,
    .free = vPortFree,
    .enter_critical = vPortEnterCritical,
    .exit_critical = vPortExitCritical,
    /* relative to the real world time */
    .realtime_init = NULL,
    .realtime_trim = NULL,
    .get_realtime_ms = NULL,
    .get_realtime = NULL,
};



