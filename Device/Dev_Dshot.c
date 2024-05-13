#include "Dev_Dshot.h"
#include <math.h>

__attribute__((weak)) void *DShot_Malloc(uint32_t size){return NULL;}
__attribute__((weak)) void DShot_Free(void *ptr){return;}
__attribute__((weak)) bool DShot_Port_Init(void *obj, uint32_t prescaler, void *time_ins, uint32_t time_ch, void *pin, uint8_t dma, uint8_t stream){return false;}
__attribute__((weak)) bool DShot_Port_DeInit(void *obj){return false;}
__attribute__((weak)) void DShot_Port_Trans(void *obj){return;}
__attribute__((weak)) uint32_t DShot_Get_Timer_CLKFreq(void *obj){return 0;}

/* external function */
static bool DevDshot_Init(DevDshotObj_TypeDef *obj, void *timer_ins, uint32_t ch, void *pin, uint8_t dma, uint8_t stream);
static bool DevDshot_DeInit(DevDshotObj_TypeDef *obj);
static void DevDshot_Control(DevDshotObj_TypeDef *obj, uint16_t value);
static void DevDshot_Command(DevDshotObj_TypeDef *obj, uint16_t cmd);

DevDshot_TypeDef DevDshot = {
    .init = DevDshot_Init,
    .de_init = DevDshot_DeInit,
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

static bool DevDshot_DeInit(DevDshotObj_TypeDef *obj)
{
    if (obj)
        return DShot_Port_DeInit(obj);

    return false;
}

static bool DevDshot_Init(DevDshotObj_TypeDef *obj,
                          void *timer_ins,
                          uint32_t ch,
                          void *pin,
                          uint8_t dma,
                          uint8_t stream)
{
    float DShot_Timer_ClkFreq = (float)DShot_Get_Timer_CLKFreq(obj);
    uint32_t prescaler = 0;

    if (!obj || (DShot_Timer_ClkFreq == 0))
        return false;

    prescaler = lrintf(DShot_Timer_ClkFreq / DevDshot_GetType_Clock(obj->type) + 0.01f) - 1;

#if defined STM32H743xx
    obj->pwm_obj.tim_hdl = DShot_Malloc(TIM_HandleType_Size);
    if(obj->pwm_obj.tim_hdl == NULL)
    {
        DShot_Free(obj->pwm_obj.tim_hdl);
        return false;
    }

    obj->pwm_obj.dma_hdl = DShot_Malloc(TIM_DMA_HandleType_Size);
    if(obj->pwm_obj.dma_hdl == NULL)
    {
        DShot_Free(obj->pwm_obj.tim_hdl);
        DShot_Free(obj->pwm_obj.dma_hdl);
        return false;
    }
#endif

    if ((obj->type < DevDshot_150) || (obj->type > DevDshot_600))
    {
        obj->type = DevDshot_300;
    }

    if (!DShot_Port_Init(obj, prescaler, timer_ins, ch, pin, dma, stream))
        return false;

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

    DShot_Port_Trans(obj);
}

static void DevDshot_Command(DevDshotObj_TypeDef *obj, uint16_t cmd)
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

    DShot_Port_Trans(obj);
}
