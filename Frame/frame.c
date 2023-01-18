/* coder: 8_B!T0
 * frame format down below
 * < 0 header_1>< 1 header_2>< 2 type>< 3 dir_type>< 4 size>< 5 payload>< end crc16>
 */
#include "frame.h"

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
                    break;

                case Frame_Type_Receiver:
                    break;

                case Frame_Type_IMU:
                    break;

                default:
                    return Frame_Decode_Type_Error;
            }

            /* update frame communicate time */
        }
        else
            return Frame_Decode_Header_Error;
    }

    return Frame_Decode_RecData_Error;
}
