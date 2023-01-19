/* coder: 8_B!T0
 * frame format down below
 * < 0 header_1>< 1 header_2>< 2 type>< 3 dir_type>< 4 size>< 5 payload>< end crc16>
 * crc gets from first byte payload to last byte in the payload
 */
#include "frame.h"
#include "util.h"

__attribute__((weak)) uint32_t Frame_Get_Runtime(void) { return 0; };
__attribute__((weak)) int8_t Frame_ChannelSetting_Callback(const Frame_ChannelSetting_TypeDef rec_data){};
__attribute__((weak)) int32_t Frame_Protocol_Pack(uint8_t *p_data, uint32_t size){};

/* internal function */
static void Frame_Update_ChannelSetting(const Frame_ChannelSetting_TypeDef rec_data);

/* internal function */
Frame_Monitor_TypeDef frame_monitor = {
    .err_cnt = 0,
    .receiver_setting_err_cnt = 0,
    .receiver_setting_cnt = 0,
    .update_rt = 0,
};

Frame_Decode_ErrorCode_List Frame_Decode(uint8_t *p_data, uint16_t size)
{
    Frame_Format_TypeDef frame;
    uint8_t *payload = NULL;
    uint16_t crc_in = 0;

    memset(&frame, 0, sizeof(Frame_Format_TypeDef));

    if (p_data && (size > 0))
    {
        memcpy(&frame, p_data, sizeof(Frame_Format_TypeDef));

        /* check header */
        if ((frame.header_1 == FRAEM_HEADER_1) &&
            (frame.header_2 == FRAEM_HEADER_2))
        {
            switch (frame.type)
            {
            case Frame_Type_HeartBeat:
                if ((frame.dir != FRAME_HEADER_DIR) ||
                    (frame.size != FRAME_HEARTBEAT_SIZE))
                {
                    frame_monitor.err_cnt++;
                    return Frame_Decode_HeartBeat_Error;
                }

                payload = p_data + sizeof(Frame_Format_TypeDef);

                if (*((uint16_t *)(payload)) != FRAME_ENDER)
                {
                    frame_monitor.err_cnt++;
                    return Frame_Decode_HeartBeat_Error;
                }
                break;

            case Frame_Type_Receiver:
                if (frame.dir == Frame_ReceiverChannel_Set)
                {
                    if (frame.size == sizeof(Frame_ChannelSetting_TypeDef))
                    {
                        payload = p_data + sizeof(Frame_Format_TypeDef);
                        crc_in = *((uint16_t *)(payload + frame.size));

                        /* check crc */
                        if(Common_CRC16(payload, frame.size) == crc_in)
                        {
                            Frame_ChannelSetting_TypeDef ChannelSetting_Tmp;
                            memset(&ChannelSetting_Tmp, 0, sizeof(ChannelSetting_Tmp));
                            Frame_Update_ChannelSetting(ChannelSetting_Tmp);
                        }
                    }
                    else
                    {
                        frame_monitor.err_cnt++;
                        frame_monitor.receiver_setting_err_cnt++;
                    }
                }
                break;

            case Frame_Type_IMU:
                break;

            default:
                frame_monitor.err_cnt++;
                return Frame_Decode_Type_Error;
            }

            /* update frame communicate time */
            frame_monitor.update_rt = Frame_Get_Runtime();
        }
        else
            return Frame_Decode_Header_Error;
    }

    return Frame_Decode_RecData_Error;
}

static void Frame_Update_ChannelSetting(const Frame_ChannelSetting_TypeDef rec_data)
{
    Frame_OutputStream_TypeDef out_stream;
    int8_t ack_state = false;
    memset(&out_stream, 0, sizeof(out_stream));

    ack_state = Frame_ChannelSetting_Callback(rec_data);

    out_stream.format.header_1 = FRAEM_HEADER_1;
    out_stream.format.header_2 = FRAEM_HEADER_2;
    out_stream.format.type = Frame_Type_Receiver;
    out_stream.format.dir = Frame_ReceiverChannel_Set_Ack;
    out_stream.format.size = FRAME_ACK_SIZE;

    out_stream.ptr = &ack_state;
    out_stream.crc16 = FRAME_ENDER;

    /* send frame ack */
    Frame_Protocol_Pack();
}

