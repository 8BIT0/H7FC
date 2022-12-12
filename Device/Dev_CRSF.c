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
    uint8_t frame_size = 0;

    if ((obj == NULL) || (p_data == NULL) || (len <= 3))
        return false;

    obj->frame.device_addr = p_data[0];
    obj->frame.frame_size = p_data[1];
    obj->frame.type = p_data[2];

    if(obj->frame.frame_size > CRSF_FRAME_SIZE_MAX)
        return false;

    memcpy(obj->frame.type, p_data + 3, len - 3);

    /* check crc first */
    if(crsf_crc8(p_data, len) == p_data[len - 1])
    {
    /* check frame type */
    switch(obj->frame.type)
    {
        case CRSF_FRAMETYPE_GPS:
            break;

        case CRSF_FRAMETYPE_BATTERY_SENSOR:
            break;

        case CRSF_FRAMETYPE_LINK_STATISTICS:
            if ((CRSF_ADDRESS_FLIGHT_CONTROLLER == obj->frame.device_addr) &&
                ((CRSF_FRAME_ORIGIN_DEST_SIZE + CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE) == obj->frame.frame_size))
            {

            }
            break;

        case CRSF_FRAMETYPE_OPENTX_SYNC:
            break;

        case CRSF_FRAMETYPE_RADIO_ID:
            break;

        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            if(CRSF_ADDRESS_FLIGHT_CONTROLLER == obj->frame.device_addr)
            {

            }
            break;

        case CRSF_FRAMETYPE_ATTITUDE:
            break;

        case CRSF_FRAMETYPE_FLIGHT_MODE:
            break;

        // Extended Header Frames, range: 0x28 to 0x96
        case CRSF_FRAMETYPE_DEVICE_PING:
            break;

        case CRSF_FRAMETYPE_DEVICE_INFO:
            break;

        case CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY:
            break;

        case CRSF_FRAMETYPE_PARAMETER_READ:
            break;

        case CRSF_FRAMETYPE_PARAMETER_WRITE:
            break;

        case CRSF_FRAMETYPE_COMMAND:
            break;

        // MSP commands
        case CRSF_FRAMETYPE_MSP_REQ:   // response request using msp sequence as command
            break;

        case CRSF_FRAMETYPE_MSP_RESP:  // reply with 58 byte chunked binary
            break;

        case CRSF_FRAMETYPE_MSP_WRITE: // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
            break;

        default:
            return false;
    }
    }

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
