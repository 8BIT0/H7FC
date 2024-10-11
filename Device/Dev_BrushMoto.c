#include "Dev_BrushMoto.h"

#define BRUSH_TMR_CLK_FREQ  1000000
#define BRUSH_TMR_PERIOD    BRUSH_MAX_THROTTLE

__attribute__((weak)) bool Brush_Port_Init(void *obj, uint32_t prescaler, uint32_t autoreload, void *time_ins, uint32_t time_ch, void *pin){return false;}
__attribute__((weak)) bool Brush_Port_DeInit(void *obj){return false;}
__attribute__((weak)) void Brush_Port_Trans(void *obj, uint16_t val){return;}
__attribute__((weak)) uint32_t Brush_Get_Timer_CLKFreq(void *tmr_obj){return 0;}

/* external function */
static bool DevBrushMoto_Init(DevBrushMotoObj_TypeDef *obj, void *timer_ins, uint32_t ch, void *pin);
static bool DevBrushMoto_Deinit(DevBrushMotoObj_TypeDef *obj);
static void DevBrushMoto_Control(DevBrushMotoObj_TypeDef *obj, uint16_t val);

DevBrushMoto_TypeDef DevBrushMoto = {
    .init = DevBrushMoto_Init,
    .de_init = DevBrushMoto_Deinit,
    .control = DevBrushMoto_Control,
};

static bool DevBrushMoto_Init(DevBrushMotoObj_TypeDef *obj, void *timer_ins, uint32_t ch, void *pin)
{
    uint32_t tmr_clk = 0;
    uint32_t prescaler = 0;
    uint32_t autoreload = 0;

    if ((obj == NULL) || \
        (timer_ins == NULL))
        return false;

    tmr_clk = Brush_Get_Timer_CLKFreq(obj);
    if (tmr_clk == 0)
        return false;

    prescaler = tmr_clk / BRUSH_TMR_CLK_FREQ;
    autoreload = BRUSH_TMR_PERIOD;

    return Brush_Port_Init(obj, prescaler, autoreload, timer_ins, ch, pin);
}

static bool DevBrushMoto_Deinit(DevBrushMotoObj_TypeDef *obj)
{
    if ((obj == NULL) || \
        (obj->p_timer_obj == NULL))
        return false;

    return Brush_Port_DeInit(obj);
}

static void DevBrushMoto_Control(DevBrushMotoObj_TypeDef *obj, uint16_t val)
{
    if ((obj == NULL) || \
        (obj->p_timer_obj == NULL))
        return;

    obj->ctl_val = val;
    Brush_Port_Trans(obj, val);
}

