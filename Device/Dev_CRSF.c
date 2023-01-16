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

static bool DevCrsf_Init(DevCRSFObj_TypeDef *obj);
static uint8_t DevCRSF_Decode(DevCRSFObj_TypeDef *obj, uint8_t *p_data, uint16_t len);
static crsf_channels_t DevCRSF_Get_Channel(DevCRSFObj_TypeDef *obj);
static crsf_LinkStatistics_t DevCESF_Get_Statistics(DevCRSFObj_TypeDef *obj);
static uint8_t DevCRSF_FIFO_In(DevCRSFObj_TypeDef *obj, uint8_t *p_data, uint8_t arg);

DevCRSF_TypeDef DevCRSF = {
    .init = DevCrsf_Init,
    .decode = DevCRSF_FIFO_In,
    .get_channel = DevCRSF_Get_Channel,
    .get_statistics = DevCESF_Get_Statistics,
};

static uint8_t crsf_crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9};

static uint8_t crsf_crc8(uint8_t *ptr, uint16_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        crc = crsf_crc8tab[crc ^ *ptr++];
    }
    return crc;
}

static bool DevCrsf_Init(DevCRSFObj_TypeDef *obj)
{
    if (obj == NULL)
        return false;

    memset(obj, 0, sizeof(DevCRSFObj_TypeDef));

    obj->state = CRSF_State_LinkDown;
    obj->rec_cnt = 0;
    obj->rec_stage = CRSF_Stage_Header;

    return true;
}

static uint8_t DevCRSF_FIFO_In(DevCRSFObj_TypeDef *obj, uint8_t *p_data, uint8_t arg)
{
    if (obj)
    {
        switch ((uint8_t)(obj->rec_stage))
        {
        case CRSF_Stage_Header:
            if (*p_data == CRSF_ADDRESS_FLIGHT_CONTROLLER)
            {
                obj->rec_stage = CRSF_Stage_Size;

                obj->frame.buff[obj->rec_cnt] = *p_data;
                obj->rec_cnt++;
            }
            else
                obj->rec_cnt = 0;
            break;

        case CRSF_Stage_Size:
            if (*p_data <= CRSF_PAYLOAD_SIZE_MAX)
            {
                obj->rec_stage = CRSF_Stage_Type;

                obj->frame.buff[obj->rec_cnt] = *p_data;
                obj->rec_cnt++;
            }
            else
            {
                obj->rec_cnt = 0;
                obj->rec_stage = CRSF_Stage_Header;
            }
            break;

        case CRSF_Stage_Type:
            break;
        }
    }
}

/* serial receiver receive callback */
static uint8_t DevCRSF_Decode(DevCRSFObj_TypeDef *obj, uint8_t *p_data, uint16_t len)
{
    uint8_t frame_size = 0;

    // if ((obj == NULL) || (p_data == NULL) || (len <= 3))
    //     return false;

    // if ((obj->frame.frame_size == 0) || (obj->frame.frame_size > CRSF_FRAME_SIZE_MAX))
    //     return false;

    // /* check crc first */
    // if (crsf_crc8(p_data, obj->frame.frame_size) == obj->frame.data[obj->frame.payload_size - 1])
    // {
    //     /* check frame type */
    //     switch (obj->frame.type)
    //     {
    //     case CRSF_FRAMETYPE_LINK_STATISTICS:
    //         if ((CRSF_ADDRESS_FLIGHT_CONTROLLER == obj->frame.device_addr) &&
    //             ((CRSF_FRAME_ORIGIN_DEST_SIZE + CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE) == obj->frame.frame_size))
    //         {
    //             const crsf_LinkStatistics_t *stats = (const crsf_LinkStatistics_t *)&(obj->frame.data);
    //             memcpy(&obj->statistics, stats, sizeof(crsf_LinkStatistics_t));
    //             obj->state = CRSF_State_LinkUp;

    //             return CRSF_FRAMETYPE_LINK_STATISTICS;
    //         }
    //         break;

    //     case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
    //         if (CRSF_ADDRESS_FLIGHT_CONTROLLER == obj->frame.device_addr)
    //         {
    //             memset(&obj->channel, 0, sizeof(obj->channel));
    //             const crsf_channels_t *channel_val_ptr = (crsf_channels_t *)&(obj->frame.data);
    //             memcpy(&obj->channel, channel_val_ptr, sizeof(crsf_channels_t));
    //             obj->state = CRSF_State_LinkUp;

    //             return CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
    //         }
    //         break;

    //     default:
    //         return CRSF_DECODE_ERROR;
    //     }
    // }

    return CRSF_DECODE_ERROR;
}

static crsf_channels_t DevCRSF_Get_Channel(DevCRSFObj_TypeDef *obj)
{
    crsf_channels_t channel;

    memset(&channel, 0, sizeof(channel));

    if (obj)
        memcpy(&channel, &obj->channel, sizeof(crsf_channels_t));

    return channel;
}

static crsf_LinkStatistics_t DevCESF_Get_Statistics(DevCRSFObj_TypeDef *obj)
{
    crsf_LinkStatistics_t statistics;

    memset(&statistics, 0, sizeof(crsf_LinkStatistics_t));

    if (obj)
        memcpy(&statistics, &obj->statistics, sizeof(crsf_LinkStatistics_t));

    return statistics;
}
