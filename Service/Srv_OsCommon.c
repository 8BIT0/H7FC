#include "Srv_OsCommon.h"

/* external function */

SrvOsCommon_TypeDef SrvOsCommon = {
    .get_os_ms = osKernelSysTick,
    .delay_ms = osDelay,
    .malloc = pvPortMalloc,
    .free = vPortFree,
    .realtime_init = NULL,
    .realtime_trim = NULL,
    .get_realtime_ms = NULL,
    .get_realtime = NULL,
};



