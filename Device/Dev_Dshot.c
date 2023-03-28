#include "Dev_Dshot.h"
#include <math.h>

/* external function */
static bool DevDshot_Init(DevDshotObj_TypeDef *obj, void *timer_ins, uint32_t ch, BspGPIO_Obj_TypeDef pin, uint8_t dma, uint8_t stream);
static void DevDshot_Control(DevDshotObj_TypeDef *obj, uint16_t value);
static void DevDshot_Command(DevDshotObj_TypeDef *obj, uint8_t cmd);

DevDshot_TypeDef DevDshot = {
    .init = DevDshot_Init,
    .command = DevDshot_Command,
    .control = DevDshot_Control,
};

static uint32_t DevDshot_GetType_Clock(DevDshotType_List type)
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
    uint32_t prescaler = lrintf((float)DSHOT_TIMER_CLK_HZ / DevDshot_GetType_Clock(obj->type) + 0.01f) - 1;

    if (!obj)
        return false;

    if ((obj->type < DevDshot_150) || (obj->type > DevDshot_600))
    {
        obj->type = DevDshot_300;
    }

    if (!BspTimer_PWM.init(&obj->pwm_obj, timer_ins, ch, pin, dma, stream, (uint32_t)obj->ctl_buf, DSHOT_DMA_BUFFER_SIZE))
        return false;

    BspTimer_PWM.set_prescaler(&obj->pwm_obj, prescaler);
    BspTimer_PWM.set_autoreload(&obj->pwm_obj, MOTOR_BITLENGTH);

    BspTimer_PWM.start_pwm(&obj->pwm_obj);

    return true;
}

static uint16_t DevDshot_Prepare_Packet(const uint16_t value, int8_t requestTelemetry)
{
    uint16_t packet = (value << 1) | (requestTelemetry ? 1 : 0);

    // compute checksum
    int csum = 0;
    int csum_data = packet;
    for (int i = 0; i < 3; i++)
    {
        csum ^= csum_data; // xor data by nibbles
        csum_data >>= 4;
    }

    csum &= 0xf;
    // append checksum
    packet = (packet << 4) | csum;
    return packet;
}

static void DevDshot_Control(DevDshotObj_TypeDef *obj, uint16_t value)
{
    uint16_t packet;
    bool dshot_telemetry = false;

    if (!obj)
        return;

    if (value > DSHOT_MAX_THROTTLE)
        value = DSHOT_MAX_THROTTLE;

    if (value < DSHOT_MIN_THROTTLE)
        value = DSHOT_LOCK_THROTTLE;

    packet = DevDshot_Prepare_Packet(value, dshot_telemetry);

    for (int i = 0; i < 16; i++)
    {
        obj->ctl_buf[i] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
        packet <<= 1;
    }

    obj->ctl_buf[16] = 0;
    obj->ctl_buf[17] = 0;

    obj->pwm_obj.buffer_addr = (uint32_t)obj->ctl_buf;
    obj->pwm_obj.buffer_size = DSHOT_DMA_BUFFER_SIZE;

    BspTimer_PWM.dma_trans(&obj->pwm_obj);
}

static void DevDshot_Command(DevDshotObj_TypeDef *obj, uint8_t cmd)
{
    uint16_t packet;
    bool dshot_telemetry = false;

    if (!obj || cmd >= DSHOT_MIN_THROTTLE)
        return;

    packet = DevDshot_Prepare_Packet(cmd, dshot_telemetry);

    for (int i = 0; i < 16; i++)
    {
        obj->ctl_buf[i] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
        packet <<= 1;
    }

    obj->ctl_buf[16] = 0;
    obj->ctl_buf[17] = 0;

    BspTimer_PWM.dma_trans(&obj->pwm_obj);
}
