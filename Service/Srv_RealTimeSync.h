#ifndef __SRV_REALTIMESYNC_H
#define __SRV_REALTIMESYNC_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "Bsp_Timer.h"

typedef enum
{
    RealTime_Invalid = 0,
    RealTime_Solid,
    RealTime_Fuzzy,
}SrvRealTime_SolidState_TypeList;

typedef struct
{
    uint8_t solid_state; /* use as SrvRealTime_SolidState_TypeList */

    uint16_t week;
    uint32_t week_sec;
    uint32_t milli_sec;
    
    /* use uint64_t stand for realtime data but in some of micor unit uint64_t type may cause some unexpect risk */
    uint32_t realtime_M32;  /* high 4byte */
    uint32_t realtime_L32;  /* low  4byte */
}SrvRealTimeObj_TypeDef;

typedef struct
{
    uint32_t tick_base;
    uint32_t pps_rec_cnt;
    BspTimerTickObj_TypeDef tick_obj;

}SrvRealTimeMonitor_TypeDef;

typedef struct
{
    bool (*init)();
}SrvRealTime_TypeDef;

#endif
