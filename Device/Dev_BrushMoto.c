/*
 *  Author:8_B!T0
 *  still in developping
 */
#include "Dev_BrushMoto.h"
#include <math.h>

#define BRUSH_TMR_CLK_FREQ  5000000
#define BRUSH_TMR_PERIOD    BRUSH_MAX_THROTTLE

__attribute__((weak)) bool Brush_Port_Init(void *obj, uint32_t prescaler, void *time_ins, uint32_t time_ch, void *pin, int8_t dma, int8_t stream){return false;}
__attribute__((weak)) bool Brush_Port_DeInit(void *obj){return false;}
__attribute__((weak)) void Brush_Port_Trans(void *obj){return;}
__attribute__((weak)) uint32_t Brush_Get_Timer_CLKFreq(void *tmr_obj){return 0;}

/* external function */
static bool DevBrushMoto_Init(DevBrushMotoObj_TypeDef *obj, void *timer_ins, uint32_t ch, void *pin, int8_t dma, int8_t stream);
static bool DevBrushMoto_Deinit(DevBrushMotoObj_TypeDef *obj);
static void DevBrushMoto_Control(DevBrushMotoObj_TypeDef *obj, uint32_t val);

DevBrushMoto_TypeDef DevBrushMoto = {
    .init = DevBrushMoto_Init,
    .de_init = DevBrushMoto_Deinit,
    .control = DevBrushMoto_Control,
};

static bool DevBrushMoto_Init(DevBrushMotoObj_TypeDef *obj, void *timer_ins, uint32_t ch, void *pin, int8_t dma, int8_t stream)
{
    uint32_t tmr_clk = 0;
    uint32_t prescaler = 0;

    if ((obj == NULL) || \
        (timer_ins == NULL))
        return false;

    tmr_clk = Brush_Get_Timer_CLKFreq(obj);
    if (tmr_clk == 0)
        return false;

    prescaler = lrintf(tmr_clk / BRUSH_TMR_CLK_FREQ);

    return Brush_Port_Init(obj, prescaler, timer_ins, ch, pin, dma, stream);
}

static bool DevBrushMoto_Deinit(DevBrushMotoObj_TypeDef *obj)
{
    if ((obj == NULL) || \
        (obj->p_timer_obj == NULL))
        return false;

    return Brush_Port_DeInit(obj);
}

static void DevBrushMoto_Control(DevBrushMotoObj_TypeDef *obj, uint32_t val)
{
    if ((obj == NULL) || \
        (obj->p_timer_obj == NULL))
        return;

    obj->ctl_val[0] = val;
    obj->ctl_val[1] = 0;
    Brush_Port_Trans(obj);
}

