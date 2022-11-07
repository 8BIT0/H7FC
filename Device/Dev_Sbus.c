#include "Dev_Sbus.h"

__weak uint32_t DevSbus_Get_SysMs(void) {return 0;}

static DevSBUS_ErrorCode_List DevSBUS_Frame_Decode(uint8_t *ptr, uint16_t size, DevSBUSData_TypeDef *data);
static DevSBUS_ErrorCode_List DevSBUS_Frame_Encode(uint8_t *ptr, uint16_t size, const DevSBUSData_TypeDef obj);

DevSBUS_TypeDef DevSBUS = {
    .decode = DevSBUS_Frame_Decode,
    .encode = DevSBUS_Frame_Encode,
};

static DevSBUS_ErrorCode_List DevSBUS_Frame_Decode(uint8_t *ptr, uint16_t size, DevSBUSData_TypeDef *data)
{
    if((ptr == NULL) || (size < SBUS_FRAME_BYTE_SIZE) || (data == NULL))
        return DevSBUS_Error_Obj;

    memset(data, NULL, sizeof(DevSBUSData_TypeDef));
    data->valid = false;

    if ((ptr[0] == SBUS_FRAME_HEADER) && (SBUS_FRAME_BYTE_SIZE == size))
    {
        data->val[0] = ((ptr[1] | ptr[2] << 8) & SBUS_DECODE_MASK);
        data->val[1] = ((ptr[2] >> 3 | ptr[3] << 5) & SBUS_DECODE_MASK);
        data->val[2] = ((ptr[3] >> 6 | ptr[4] << 2 | ptr[5] << 10) & SBUS_DECODE_MASK);
        data->val[3] = ((ptr[5] >> 1 | ptr[6] << 7) & SBUS_DECODE_MASK);
        data->val[4] = ((ptr[6] >> 4 | ptr[7] << 4) & SBUS_DECODE_MASK);
        data->val[5] = ((ptr[7] >> 7 | ptr[8] << 1 | ptr[9] << 9) & SBUS_DECODE_MASK);
        data->val[6] = ((ptr[9] >> 2 | ptr[10] << 6) & SBUS_DECODE_MASK);
        data->val[7] = ((ptr[10] >> 5 | ptr[11] << 3) & SBUS_DECODE_MASK);
        data->val[8] = ((ptr[12] | ptr[13] << 8) & SBUS_DECODE_MASK);
        data->val[9] = ((ptr[13] >> 3 | ptr[14] << 5) & SBUS_DECODE_MASK);
        data->val[10] = ((ptr[14] >> 6 | ptr[15] << 2 | ptr[16] << 10) & SBUS_DECODE_MASK);
        data->val[11] = ((ptr[16] >> 1 | ptr[17] << 7) & SBUS_DECODE_MASK);
        data->val[12] = ((ptr[17] >> 4 | ptr[18] << 4) & SBUS_DECODE_MASK);
        data->val[13] = ((ptr[18] >> 7 | ptr[19] << 1 | ptr[20] << 9) & SBUS_DECODE_MASK);
        data->val[14] = ((ptr[20] >> 2 | ptr[21] << 6) & SBUS_DECODE_MASK);
        data->val[15] = ((ptr[21] >> 5 | ptr[22] << 3) & SBUS_DECODE_MASK);

        data->valid = true;
        data->time_stamp = DevSbus_Get_SysMs();
        return DevSBUS_NoError;
    }

    return DevSBUS_Error_Frame;
}

static DevSBUS_ErrorCode_List DevSBUS_Frame_Encode(uint8_t *ptr, uint16_t size, const DevSBUSData_TypeDef obj)
{
    if((ptr == NULL) || (size < SBUS_FRAME_BYTE_SIZE))
        return DevSBUS_Error_Obj;


}

