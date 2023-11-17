#include "Srv_RealTimeSync.h"

/*
 *  need to use two high accurately timer (32bit timer)
 *  one timer for moniting another for kernel tick
 */

static bool SrvRealTimeSync_Init()
{
    /* timer init first */
    BspTimer_Tick.init();

    return false;
}

