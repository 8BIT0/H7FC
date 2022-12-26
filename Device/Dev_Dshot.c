#include "Bsp_Timer.h"
#include "Dev_Dshot.h"
#include <math.h>

static uint16_t DevDshot_GetType_Clock(DevDshotType_List type)
{
    switch (type)
    {
    case DevDshot_150:
        return DSHOT150_CLK_HZ;

    case DevDshot_300:
        return DSHOT300_CLK_HZ;

    case DevDshot_600:
        return DSHOT600_CLK_HZ;

    default:
        return DSHOT300_CLK_HZ;
    }
}

static bool DevDshot_Init(DevDshotObj_TypeDef *obj,
                          TIM_TypeDef *instance,
                          uint32_t ch,
                          BspGPIO_Obj_TypeDef pin,
                          uint8_t dma,
                          uint8_t stream)
{
    uint16_t dshot_clk = 0;
    uint16_t prescaler = 0;

    if (!obj)
        return false;

    if ((obj->type < DevDshot_150) || (obj->type > DevDshot_600))
    {
        obj->type = DevDshot_300;
    }

    dshot_clk = DevDshot_GetType_Clock(obj->type);

    BspTimer_PWM.init(&obj->pwm_obj, instance, ch, pin, dma, stream, (uint32_t)obj->ctl_buf, MOTOR_BITLENGTH);

    return true;
}
