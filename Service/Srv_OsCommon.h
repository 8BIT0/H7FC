#ifndef __SRV_OSCOMMON_H
#define __SRV_OSCOMMON_H

#include "cmsis_os.h"
#include <stdint.h>
#include <stdbool.h>
#include "semaphore.h"

typedef struct
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint16_t ms;
}SrvOs_RealTime_TypeDef;

typedef SrvOs_RealTime_TypeDef SrvOs_GPSTime_TypeDef;

typedef struct
{
    uint32_t (*get_os_ms)(void);
    int32_t (*delay_ms)(uint32_t ms);
    int32_t (*precise_delay)(uint32_t *p_time, uint32_t ms);

    bool (*realtime_init)(void);
    bool (*realtime_trim)(SrvOs_RealTime_TypeDef timeobj);
    uint32_t (*get_realtime_ms)(void);
    SrvOs_RealTime_TypeDef (*get_realtime)(void);

    void *(*malloc)(uint16_t size);
    void (*free)(void *ptr);

    void (*enter_critical)(void);
    void (*exit_critical)(void);
}SrvOsCommon_TypeDef;

extern SrvOsCommon_TypeDef SrvOsCommon;

#endif
