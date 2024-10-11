#include "Dev_BrushMoto.h"

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
    if ((obj == NULL) || \
        (timer_ins == NULL))
        return false;

    return false;
}

static bool DevBrushMoto_Deinit(DevBrushMotoObj_TypeDef *obj)
{
    if (obj == NULL)
        return false;

    return false;
}

static void DevBrushMoto_Control(DevBrushMotoObj_TypeDef *obj, uint16_t val)
{
    if (obj == NULL)
        return;
}

