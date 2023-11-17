#include "Srv_RealTimeSync.h"

/*
 *  need to use two high accurately timer (32bit timer)
 *  one timer for moniting another for kernel tick
 */

static SrvRealTimeMonitor_TypeDef SrvRealTime_Monitor = {
    .init = false;
};

static bool SrvRealTimeSync_Init()
{
    /* realtime module is unqiue can`t be multiple in a single drone */
    if(SrvRealTime_Monitor.init)
        return false;

    /* timer init first */
    BspTimer_Tick.init();

    SrvRealTime_Monitor.init = true;

    return false;
}

