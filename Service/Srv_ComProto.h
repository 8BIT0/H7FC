#ifndef __SRV_COMPROTO_H
#define __SRV_COMPROTO_H

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "MAVLink\common\common.h"

typedef bool (*ComProto_Callback)(uint8_t *p_data, uint32_t len);

typedef enum
{
    SrvComProto_Type_Cus = 0,
    SrvComProto_Type_MAV,
} SrvComProto_Type_List;

typedef enum
{
    SrvComProto_FrameOut = 1,
    SrvComProto_FrameIn,
} SrvComProto_IOType_List;

typedef enum
{

} SrvComProto_MAVMsg_List;

typedef union
{
    struct section
    {
        uint32_t msg_altitude : 1;
        uint32_t msg_attitude_quaternion : 1;
        uint32_t msg_attitude : 1;
        uint32_t msg_battery_status : 1;
        uint32_t msg_raw_imu : 1;
        uint32_t msg_raw_pressure : 1;
        uint32_t msg_rc_channels_raw : 1;
        uint32_t msg_rc_channels_scaled : 1;
        uint32_t msg_scaled_imu : 1;
        uint32_t msg_scaled_imu2 : 1;
        uint32_t msg_scaled_pressure : 1;
        uint32_t msg_sys_status : 1;

        uint32_t res_bit : 16;

        uint32_t reserve[3];
    };

    uint32_t val[4];
} SrvComProto_MavReg_TypeDef;

typedef struct
{
    uint8_t *p_buf;
    uint16_t size;
} SrvComProto_Stream_TypeDef;

typedef struct
{
    SrvComProto_Type_List Proto_Type;
    SrvComProto_MavReg_TypeDef MavTx_Reg;
    SrvComProto_MavReg_TypeDef MavRx_Reg;

    uint32_t proto_msg_attitude_cnt;
    uint32_t proto_msg_raw_imu_cnt;
    uint32_t proto_msg_scaled_imu_cnt;
    uint32_t proto_msg_scaled_imu2_cnt;
} SrvComProto_Monitor_TypeDef;

typedef struct
{
    uint32_t msg_id;
    SrvComProto_IOType_List IOType;
    uint16_t freq;

    uint8_t *p_data;
    uint16_t data_size;

    uint8_t *p_buf;
    uint16_t buf_size;
} SrvComProto_MsgInfo_TypeDef;

typedef struct
{
    void (*init)(SrvComProto_Type_List type, uint8_t *arg);
    bool (*create_msg)(SrvComProto_IOType_List IOType, );
    void (*set_decode_callback)();

    bool (*mav_msg_decode)(uint8_t *p_buf, uint32_t len);
    bool (*mav_msg_proto)(SrvComProto_MsgInfo_TypeDef msg, ComProto_Callback proto_cb);
}

#endif
