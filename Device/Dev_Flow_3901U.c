#include "Dev_Flow_3901U.h"

/* noticed: sensor max update frequence is about 66Hz */

#define FLOW3901U_Frame_Header      0xFE
#define FLOW3901U_Payload_Size      0x04
#define FLOW3901U_General_Ender     0xAA
#define FLOW3901U_YawMode_Ender     0xBB
#define FLOW3901U_Frame_Size        sizeof(Dev3901U_Frame_TypeDef)

#define FLOW3901U_Max_UpdateFreq    66

typedef struct
{
    uint8_t header;
    uint8_t byte;
    uint8_t data[4];
    uint8_t check_sum;
    uint8_t env_quality;
    uint8_t ender;
} Dev3901U_Frame_TypeDef;

/* internal function */
static uint8_t DevFlow3901U_GetCheckSum(uint8_t *p_data);

/* external function */
static uint8_t DevFlow3901_Init(DevFlow3901Obj_TypeDef *obj);
static void DevFLow3901U_Decode_Callback(DevFlow3901Obj_TypeDef *obj, uint8_t *p_buf, uint16_t len);
static DevFlow3901UData_TypeDef DevFlow3901U_GetData(DevFlow3901Obj_TypeDef *obj);

DevFlow3901_TypeDef DevFlow3901 = {
    .init = DevFlow3901_Init,
    .recv = DevFLow3901U_Decode_Callback,
    .get  = DevFlow3901U_GetData,
    .yaw_mode = NULL,
};

/* return update frequence */
static uint8_t DevFlow3901_Init(DevFlow3901Obj_TypeDef *obj)
{
    if (obj && obj->get_sys_time)
    {
        obj->init = true;
        obj->time_stamp = 0;
        obj->dis_x = 0;
        obj->dis_y = 0;

        obj->recv_cnt = 0;
        obj->recv_byte = 0;

        obj->decode_s = 0;
        obj->decode_f = 0;

        obj->yaw_mode_state = true;
        if (obj->yaw_mode_ctl)
            obj->yaw_mode_ctl(obj->yaw_mode_en);

        return FLOW3901U_Max_UpdateFreq;
    }

    return 0;
}

static void DevFLow3901U_Decode_Callback(DevFlow3901Obj_TypeDef *obj, uint8_t *p_buf, uint16_t len)
{
    Dev3901U_Frame_TypeDef frame;
    memset(&frame, 0, sizeof(Dev3901U_Frame_TypeDef));

    if (obj && obj->init && p_buf && len)
    {
        obj->recv_cnt ++;
        obj->recv_byte += len;

        if (len >= FLOW3901U_Frame_Size)
        {
            obj->accessing = false;

            /* decode directly */
            memcpy(&frame, p_buf, FLOW3901U_Frame_Size);
        
            if ((frame.header == FLOW3901U_Frame_Header) && \
                (frame.byte == FLOW3901U_Payload_Size) && \
                (frame.check_sum == DevFlow3901U_GetCheckSum(frame.data)) && \
                ((frame.ender == FLOW3901U_General_Ender) || \
                 (frame.ender == FLOW3901U_YawMode_Ender)))
            {
                obj->time_stamp = obj->get_sys_time();
                obj->dis_x = frame.data[0];
                obj->dis_x |= frame.data[1] << 8;
                obj->dis_y = frame.data[2];
                obj->dis_y = frame.data[3] << 8;
                obj->quality = frame.env_quality;

                if (frame.ender == FLOW3901U_YawMode_Ender)
                    obj->yaw_mode_state = true;

                obj->decode_s ++;
            }
        }
        else
            obj->decode_f ++;
    }
}

static DevFlow3901UData_TypeDef DevFlow3901U_GetData(DevFlow3901Obj_TypeDef *obj)
{
    DevFlow3901UData_TypeDef data_tmp;

    memset(&data_tmp, 0, sizeof(DevFlow3901UData_TypeDef));

    if (obj && obj->init)
    {
reupdate_flow_sensor:
        obj->accessing = true;
        data_tmp.time_stamp = obj->time_stamp;
        data_tmp.flow_x = obj->dis_x;
        data_tmp.flow_y = obj->dis_y;
        data_tmp.quality = obj->quality;

        /* if its updating in parse process */
        /* re_update flow pos */
        if (!obj->accessing)
            goto reupdate_flow_sensor;

        obj->accessing = false;
    }

    return data_tmp;
}

static uint8_t DevFlow3901U_GetCheckSum(uint8_t *p_data)
{
    uint8_t check_sum = 0;

    if (p_data)
    {
        for (uint8_t i = 0; i < 4; i ++)
            check_sum += p_data[i];
    }

    return check_sum;
}
