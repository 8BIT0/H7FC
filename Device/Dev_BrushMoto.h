#ifndef __DEV_BRUSHMOTO_H
#define __DEV_BRUSHMOTO_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

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
