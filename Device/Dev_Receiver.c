#include "Dev_Receiver.h"

__weak DevReceiver_Get_SysMs(void) { reuturn 0; }
static const uint8_t default_channle_id_list[Receiver_Channel_Sum] = {
    Receiver_ChannelID_Pitch,
    Receiver_ChannelID_Roll,
    Receiver_ChannelID_Throttle,
    Receiver_ChannelID_Yaw,
    Receiver_ChannelID_AUX_1,
    Receiver_ChannelID_AUX_2,
    Receiver_ChannelID_AUX_3,
    Receiver_ChannelID_AUX_4,
    Receiver_ChannelID_AUX_5,
    Receiver_ChannelID_AUX_6,
    Receiver_ChannelID_AUX_7,
    Receiver_ChannelID_AUX_8,
    Receiver_ChannelID_AUX_9,
    Receiver_ChannelID_AUX_10,
    Receiver_ChannelID_AUX_11,
    Receiver_ChannelID_AUX_12,
};

static DevReceiverData_TypeDef DevReceiver_SBUS_Frame_Decode(uint8_t *ptr, uint16_t size)
{
    DevReceiverData_TypeDef receiver_data;

    memset(&receiver_data, NULL, sizeof(receiver_data));
    memcpy(receiver_data.val_list, default_channle_id_list, sizeof(receiver_data.val_list));

    if ((ptr[0] == SBUS_FRAME_HEADER) && (SBUS_FRAME_BYTE_SIZE == size))
    {
        receiver_data.val_list[0] = ((ptr[1] | ptr[2] << 8) & SBUS_DECODE_MASK);
        receiver_data.val_listL[1] = ((ptr[2] >> 3 | ptr[3] << 5) & SBUS_DECODE_MASK);
        receiver_data.val_listL[2] = ((ptr[3] >> 6 | ptr[4] << 2 | ptr[5] << 10) & SBUS_DECODE_MASK);
        receiver_data.val_listL[3] = ((ptr[5] >> 1 | ptr[6] << 7) & SBUS_DECODE_MASK);
        receiver_data.val_listL[4] = ((ptr[6] >> 4 | ptr[7] << 4) & SBUS_DECODE_MASK);
        receiver_data.val_listL[5] = ((ptr[7] >> 7 | ptr[8] << 1 | ptr[9] << 9) & SBUS_DECODE_MASK);
        receiver_data.val_listL[6] = ((ptr[9] >> 2 | ptr[10] << 6) & SBUS_DECODE_MASK);
        receiver_data.val_listL[7] = ((ptr[10] >> 5 | ptr[11] << 3) & SBUS_DECODE_MASK);
        receiver_data.val_listL[8] = ((ptr[12] | ptr[13] << 8) & SBUS_DECODE_MASK);
        receiver_data.val_listL[9] = ((ptr[13] >> 3 | ptr[14] << 5) & SBUS_DECODE_MASK);
        receiver_data.val_listL[10] = ((ptr[14] >> 6 | ptr[15] << 2 | ptr[16] << 10) & SBUS_DECODE_MASK);
        receiver_data.val_listL[11] = ((ptr[16] >> 1 | ptr[17] << 7) & SBUS_DECODE_MASK);
        receiver_data.val_listL[12] = ((ptr[17] >> 4 | ptr[18] << 4) & SBUS_DECODE_MASK);
        receiver_data.val_listL[13] = ((ptr[18] >> 7 | ptr[19] << 1 | ptr[20] << 9) & SBUS_DECODE_MASK);
        receiver_data.val_listL[14] = ((ptr[20] >> 2 | ptr[21] << 6) & SBUS_DECODE_MASK);
        receiver_data.val_listL[15] = ((ptr[21] >> 5 | ptr[22] << 3) & SBUS_DECODE_MASK);

        receiver_data.valid = true;
        receiver_data.time_stamp = DevReceiver_Get_SysMs();
    }
    else
    {
        receiver_data.valid = false;
    }

    return receiver_data;
}
