#include "Dev_CRSF.h"

/*
 * CRSF protocol
 *
 * CRSF protocol uses a single wire half duplex uart connection.
 * The master sends one frame every 4ms and the slave replies between two frames from the master.
 *
 * 420000 baud
 * not inverted
 * 8 Bit
 * 1 Stop bit
 * Big endian
 * 420000 bit/s = 46667 byte/s (including stop bit) = 21.43us per byte
 * Max frame size is 64 bytes
 * A 64 byte frame plus 1 sync byte can be transmitted in 1393 microseconds.
 *
 * CRSF_TIME_NEEDED_PER_FRAME_US is set conservatively at 1500 microseconds
 *
 * Every frame has the structure:
 * <Device address><Frame length><Type><Payload><CRC>
 *
 * Device address: (uint8_t)
 * Frame length:   length in  bytes including Type (uint8_t)
 * Type:           (uint8_t)
 * Payload:        (uint8_t)
 * CRC:            (uint8_t)
 *
 */

static uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        crc = crsf_crc8tab[crc ^ *ptr++];
    }
    return crc;
}

static uint16_t DevCrsf_Range_Check(uint16_t channel_val)
{
    if (channel_val >= CRSF_DIGITAL_CHANNEL_MAX)
        return CRSF_DIGITAL_CHANNEL_MAX;

    if (channel_val <= CRSF_DIGITAL_CHANNEL_MIN)
        return CRSF_DIGITAL_CHANNEL_MIN;

    return channel_val;
}

static bool DevCrsf_Init(DevCRSFObj_TypeDef *obj)
{
    if (obj == NULL)
        return false;

    memset(obj, 0, sizeof(DevCRSFObj_TypeDef));
    return true;
}

static bool DevCrsf_Set_Callback(DevCRSFObj_TypeDef *obj,crsf_state_list state, CRSF_Callback cb)
{
    if(!obj)
        return false;

    switch(state)
    {
        case CRSF_State_LinkUp:
            obj->link_up_cb = cb;
            break;

        case CRSF_State_LinkDown:
            obj->link_down_cb = cb;
            break;

        case CRSF_State_TimeOut:
            obj->failsafe_cb = cb;
            break;

        default:
            return false;
    }

    return true;
}

/* serial receiver receive callback */
static bool DevCRSF_Decode(DevCRSFObj_TypeDef *obj, uint8_t *p_data, uint16_t len)
{
    if ((obj == NULL) || (p_data == NULL))
        return false;

    return true;
}

static bool DevCRSF_Callback_Proc(DevCRSFObj_TypeDef *obj, uint8_t *ptr, uint16_t size)
{
    if (obj == NULL)
        return false;

    switch (obj->state)
    {
    case CRSF_State_LinkUp:
        if(obj->link_up_cb)
            obj->link_up_cb(ptr, size);
        break;

    case CRSF_State_LinkDown:
        if(obj->link_down_cb)
            obj->link_down_cb(ptr, size);
        break;

    case CRSF_State_TimeOut:
        if(obj->failsafe_cb)
            obj->failsafe_cb(ptr, size);
        
        obj->failsafe = true;
        break;

    default:
        return false;
    }

    return true;
}
