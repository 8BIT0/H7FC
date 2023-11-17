#ifndef __SRV_OSCOMMON_H
#define __SRV_OSCOMMON_H

#include "cmsis_os.h"
#include <stdint.h>
#include <stdbool.h>

typedef HeapStats_t SrvOs_HeapStatus_TypeDef;

typedef struct
{
    uint32_t (*get_os_ms)(void);
    int32_t (*delay_ms)(uint32_t ms);
    int32_t (*precise_delay)(uint32_t *p_time, uint32_t ms);
    uint32_t (*get_systimer_current_tick)(void);
    uint32_t (*get_systimer_period)(void);
    bool (*set_systimer_tick_value)(uint32_t value);
    bool (*set_systimer_period)(uint32_t period);
    uint32_t (*systimer_disable)(void);
    uint32_t (*systimer_enable)(void);

    void *(*malloc)(uint16_t size);
    void (*free)(void *ptr);

    void (*enter_critical)(void);
    void (*exit_critical)(void);
    void (*get_heap_status)(SrvOs_HeapStatus_TypeDef *status);
}SrvOsCommon_TypeDef;

extern SrvOsCommon_TypeDef SrvOsCommon;

#endif
