#include "trace_analysiser.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug_util.h"
#include "HW_Def.h"

void trace_analysiser(uint32_t lr, uint32_t sp)
{
    DEBUG_INFO("HARD FAULT\r\n");
    TaskHandle_t p_CurTCB = 0;

    /* check mode */
    if (lr & (1 << 2))
    {
        DEBUG_INFO("THREAD MODE\r\n");

        p_CurTCB = xTaskGetCurrentTaskHandle();

        /* check thread name */
        DEBUG_INFO("Thread Name: %s\r\n", pcTaskGetName(p_CurTCB));
    }
    else
    {
        DEBUG_INFO("MAIN STACK\r\n");
    }
}

