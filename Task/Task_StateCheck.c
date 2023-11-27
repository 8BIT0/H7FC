#include "Task_StateCheck.h"

#define DEFAULT_STATE_CHECK_PERIOD 5 /* unit: ms */

/* internal vriable */
static uint32_t task_period;

void Task_StateCheck_Init(uint32_t period)
{
    if((period == 0) || (MS_PER_S / period > 500))
        task_period = DEFAULT_STATE_CHECK_PERIOD;
}

void Task_StateCheck_Core(void *arg)
{
    uint32_t pre_time = SrvOsCommon.get_os_ms();

    while(1)
    {

        SrvOsCommon.precise_delay(&pre_time, task_period);
    }
}



