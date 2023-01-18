/* coder: 8_B!T0
 * frame format down below
 * < 0 header_1>< 1 header_2>< 2 type>< 3 dir_type>< 4 size>< 5 payload>< end crc16>
 */
#include "frame.h"

__weak uint32_t Frame_Get_Runtime(void){return 0;};

/* internal function */
Frame_Monitor_TypeDef frame_monitor = {
    .err_cnt = 0,
    .receiver_setting_cnt = 0,
    .update_rt = 0,
};

Frame_Decode_ErrorCode_List Frame_Decode(uint8_t *p_data, uint16_t size)
{
    Frame_Format_TypeDef frame;
    uint8_t *payload = NULL;

    memset(&frame, 0, sizeof(Frame_Format_TypeDef));
    
    if(p_data && (size > 0))
    {
        memcpy(&frame, p_data, sizeof(Frame_Format_TypeDef));
        
        /* check header */
        if((frame.header_1 == FRAEM_HEADER_1) &&
           (frame.header_2 == FRAEM_HEADER_2))
        {
            switch(frame.type)
            {
                case Frame_Type_HeartBeat:
                    if(frame.size != FRAME_HEARTBEAT_SIZE)
                    {
                        frame_monitor.err_cnt++;
                        return Frame_Decode_HeartBeat_Error;
                    }
                    break;

                case Frame_Type_Receiver:
                    break;

                case Frame_Type_IMU:
                    break;

                default:
                    frame_monitor.err_cnt ++;
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
