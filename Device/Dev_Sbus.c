#include "Dev_Sbus.h"

static DevSBUS_ErrorCode_List DevSBUS_Frame_Decode(DevSBUSObj_TypeDef *data, uint8_t *ptr, uint16_t size);

DevSBUS_TypeDef DevSBUS = {
    .init = NULL,
    .decode = DevSBUS_Frame_Decode,
    .get_data = NULL,
};

static bool DevSBUS_Init(DevSBUSObj_TypeDef *obj)
{
    memset(obj, NULL, sizeof(DevSBUSObj_TypeDef));

    return true;
}

static DevSBUS_ErrorCode_List DevSBUS_Frame_Decode(DevSBUSObj_TypeDef *obj, uint8_t *ptr, uint16_t size)
{
    if((ptr == NULL) || (size < SBUS_FRAME_BYTE_SIZE) || (data == NULL))
        return DevSBUS_Error_Obj;

    memset(obj, NULL, sizeof(DevSBUSObj_TypeDef));

    if ((ptr[0] == SBUS_FRAME_HEADER) && (SBUS_FRAME_BYTE_SIZE == size))
    {
        obj->val[0] = ((ptr[1] | ptr[2] << 8) & SBUS_DECODE_MASK);
        obj->val[1] = ((ptr[2] >> 3 | ptr[3] << 5) & SBUS_DECODE_MASK);
        obj->val[2] = ((ptr[3] >> 6 | ptr[4] << 2 | ptr[5] << 10) & SBUS_DECODE_MASK);
        obj->val[3] = ((ptr[5] >> 1 | ptr[6] << 7) & SBUS_DECODE_MASK);
        obj->val[4] = ((ptr[6] >> 4 | ptr[7] << 4) & SBUS_DECODE_MASK);
        obj->val[5] = ((ptr[7] >> 7 | ptr[8] << 1 | ptr[9] << 9) & SBUS_DECODE_MASK);
        obj->val[6] = ((ptr[9] >> 2 | ptr[10] << 6) & SBUS_DECODE_MASK);
        obj->val[7] = ((ptr[10] >> 5 | ptr[11] << 3) & SBUS_DECODE_MASK);
        obj->val[8] = ((ptr[12] | ptr[13] << 8) & SBUS_DECODE_MASK);
        obj->val[9] = ((ptr[13] >> 3 | ptr[14] << 5) & SBUS_DECODE_MASK);
        obj->val[10] = ((ptr[14] >> 6 | ptr[15] << 2 | ptr[16] << 10) & SBUS_DECODE_MASK);
        obj->val[11] = ((ptr[16] >> 1 | ptr[17] << 7) & SBUS_DECODE_MASK);
        obj->val[12] = ((ptr[17] >> 4 | ptr[18] << 4) & SBUS_DECODE_MASK);
        obj->val[13] = ((ptr[18] >> 7 | ptr[19] << 1 | ptr[20] << 9) & SBUS_DECODE_MASK);
        obj->val[14] = ((ptr[20] >> 2 | ptr[21] << 6) & SBUS_DECODE_MASK);
        obj->val[15] = ((ptr[21] >> 5 | ptr[22] << 3) & SBUS_DECODE_MASK);

        /* decode functional byte data */

        return DevSBUS_NoError;
    }

    return DevSBUS_Error_Frame;
}


