#ifndef __SRV_OSCOMMON_H
#define __SRV_OSCOMMON_H

#include "cmsis_os.h"
#include <stdint.h>
#include <stdbool.h>

typedef HeapStats_t SrvOs_HeapStatus_TypeDef;

#define MS_PER_S 1000

typedef struct
{
    uint32_t (*get_os_ms)(void);
    int32_t (*delay_ms)(uint32_t ms);
    void (*precise_delay)(uint32_t *p_time, uint32_t ms);
    uint32_t (*get_systimer_current_tick)(void);
    uint32_t (*get_systimer_period)(void);
    uint32_t (*systimer_tick_to_us)(void);
    bool (*set_systimer_tick_value)(uint32_t value);
    bool (*set_systimer_period)(uint32_t period);
    bool (*systimer_disable)(void);
    void (*systimer_enable)(void);

    void *(*malloc)(uint32_t size);
    void (*free)(void *ptr);

    void (*enter_critical)(void);
    void (*exit_critical)(void);
    void (*get_heap_status)(SrvOs_HeapStatus_TypeDef *status);
    void (*disable_all_irq)(void);
    void (*enable_all_irq)(void);
}SrvOsCommon_TypeDef;

extern SrvOsCommon_TypeDef SrvOsCommon;

#endif
