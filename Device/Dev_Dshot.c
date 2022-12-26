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
                          void *timer_ins,
                          uint32_t ch,
                          BspGPIO_Obj_TypeDef pin,
                          uint8_t dma,
                          uint8_t stream)
{
    uint16_t prescaler = lrintf((float)DSHOT_TIMER_CLK_HZ / DevDshot_GetType_Clock(obj->type) + 0.01f) - 1;

    if (!obj)
        return false;

    if ((obj->type < DevDshot_150) || (obj->type > DevDshot_600))
    {
        obj->type = DevDshot_300;
    }

    if (!BspTimer_PWM.init(&obj->pwm_obj, timer_ins, ch, pin, dma, stream, (uint32_t)obj->ctl_buf, MOTOR_BITLENGTH))
        return false;

    BspTimer_PWM.set_prescaler(&obj->pwm_obj, prescaler);
    BspTImer_PWM.set_autoreload(&obj->pwm_obj, MOTOR_BITLENGTH);
    BspTimer_PWM.start_pwm(&obj->pwm_obj);

    return true;
}

static void DevDshot_Control(DevDshotObj_TypeDef *obj, uint16_t value)
{
    uint16_t packet;
    bool dshot_telemetry = false;

    packet = (value << 1) | (dshot_telemetry ? 1 : 0);

    // compute checksum
    uint8_t csum = 0;
    uint16_t csum_data = packet;

    for (uint8_t i = 0; i < 3; i++)
    {
        csum ^= csum_data; // xor data by nibbles
        csum_data >>= 4;
    }

    csum &= 0xf;
    packet = (packet << 4) | csum;

    for (int i = 0; i < 16; i++)
    {
        obj->ctl_buf[i] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
        packet <<= 1;
    }

    obj->ctl_buf[16] = 0;
    obj->ctl_buf[17] = 0;

    BspTimer_PWM.dma_trans(&obj->pwm_obj);
}
