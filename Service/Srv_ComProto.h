#ifndef __SRV_COMPROTO_H
#define __SRV_COMPROTO_H

#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "../System/runtime/runtime.h"
// #include "../MAVLink/common/common.h"

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
    MAV_Msg_Attitude,
    MAV_Msg_Raw_IMU,
    MAV_Msg_Raw_IMU2,
} SrvComProto_MAVMsg_List;

typedef struct
{
    uint8_t *p_buf;
    uint16_t size;
} SrvComProto_Stream_TypeDef;

typedef struct
{
    SrvComProto_Type_List Proto_Type;

    uint32_t tx_msg_attitude_cnt;
    uint32_t tx_msg_raw_imu_cnt;
    uint32_t tx_msg_scaled_imu_cnt;
    uint32_t tx_msg_scaled_imu2_cnt;
} SrvComProto_Monitor_TypeDef;

typedef struct
{
    uint32_t msg_id;
    SrvComProto_IOType_List io_type;
    uint16_t period;

    mavlink_message_t *msg_obj;

    SrvComProto_Stream_TypeDef tar_obj; /* target proto data object stream */

    SYSTEM_RunTime proto_time;

    bool in_proto;
    bool lock_proto;
} SrvComProto_MsgInfo_TypeDef;

typedef struct
{
    void (*init)(SrvComProto_Type_List type, uint8_t *arg);
    // void (*set_decode_callback)();

    bool (*mav_msg_decode)(uint8_t *p_buf, uint32_t len);
    bool (*mav_msg_obj_init)(SrvComProto_MsgInfo_TypeDef *msg, uint32_t msg_id, uint32_t period, SrvComProto_IOType_List io_dir, SrvComProto_Stream_TypeDef tar_stream);
    bool (*mav_msg_proto)(SrvComProto_MsgInfo_TypeDef msg, SrvComProto_Stream_TypeDef *com_stream, ComProto_Callback tx_cb);
} SrvComProto_TypeDef;

extern SrvComProto_TypeDef SrvComProto;

#endif
