#ifndef __DEV_BRUSHMOTO_H
#define __DEV_BRUSHMOTO_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define BRUSH_LOCK_THROTTLE 0
#define BRUSH_MIN_THROTTLE  0
#define BRUSH_IDLE_THROTTLE 100
#define BRUSH_MAX_THROTTLE  1000
#define BRUSH_RANGE         (BRUSH_MAX_THROTTLE - BRUSH_MIN_THROTTLE)

#define To_BrushObj_Ptr(x)  ((DevBrushMotoObj_TypeDef *)x)
#define To_BrushApi_Ptr(x)  ((DevBrushMoto_TypeDef *)x)
#define BrushObj_Size       sizeof(DevBrushMotoObj_TypeDef)

typedef struct
{
    bool init;
    void *p_timer_obj;
    uint16_t ctl_val;
} DevBrushMotoObj_TypeDef;

typedef struct
{
    bool (*init)(DevBrushMotoObj_TypeDef *obj, void *timer_ins, uint32_t ch, void *pin);
    bool (*de_init)(DevBrushMotoObj_TypeDef *obj);
    void (*control)(DevBrushMotoObj_TypeDef *obj, uint16_t val);
} DevBrushMoto_TypeDef;

extern DevBrushMoto_TypeDef DevBrushMoto;

#endif
