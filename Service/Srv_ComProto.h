#ifndef __SRV_COMPROTO_H
#define __SRV_COMPROTO_H

#include "Srv_OsCommon.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "../MAVLink/common/mavlink.h"

typedef bool (*ComProto_Callback)(void *arg, uint8_t *p_data, uint32_t len);
typedef uint16_t (*DataPack_Callback)(uint8_t *pck);
typedef uint32_t ComPort_Handle;

typedef enum
{
    ComFrame_Unknow = 0,
    ComFrame_CLI,
    ComFrame_MavMsg,
    ComFrame_CusMsg,
} SrvComProto_ProtoData_Type_List;

typedef enum
{
    MAV_SysID_Drone = 1,
    MAV_SysID_Cfg,
    MAV_SysID_ROS,
} SrvComProto_SysID_List;

typedef enum
{
    MAV_CompoID_Att_PIDTune_Pitch = 0,
    MAV_CompoID_Att_PIDTune_Roll,
    MAV_CompoID_Att_PIDTune_Yaw,

    MAV_CompoID_Gyr_PIDTune_X,
    MAV_CompoID_Gyr_PIDTune_Y,
    MAV_CompoID_Gyr_PIDTune_Z,
} SrvComProto_CfgComponentID_List;

typedef enum
{
    MAV_CompoID_Raw_IMU = 1,
    MAV_CompoID_Scaled_IMU,
    MAV_CompoID_Raw_Mag,
    MAV_CompoID_Scaled_Mag,
    MAV_CompoID_Attitude,
    MAV_CompoID_Exp_Attitude,
    MAV_CompoID_Altitude,
    MAV_CompoID_RC_Channel,
    MAV_CompoID_MotoCtl,
    MAV_CompoID_ServoCtl,
} SrvComProto_ComponentID_List;

typedef enum
{
    MAV_ActuatorPort_Servo = 0,
    MAV_ActuatorPort_ServoDir,
    MAV_ActuatorPort_Moto,
    MAV_ActuatorPort_MotoDir,
} SrvComProto_ActuatorPort_List;

typedef enum
{
    SrvComProto_Type_None = 0,
    SrvComProto_Type_Cus,
    SrvComProto_Type_MAV,
} SrvComProto_Type_List;

typedef struct
{
    uint8_t *p_buf;
    uint16_t size;
    uint16_t max_size;
} SrvComProto_Stream_TypeDef;

typedef struct
{
    uint8_t system_id;
    uint8_t component_id;
    uint8_t chan;
} SrvComProto_MavPackInfo_TypeDef;

typedef struct
{
    bool enable;
    uint16_t period;

    uint32_t proto_cnt;

    SrvComProto_MavPackInfo_TypeDef pck_info;
    mavlink_message_t *msg_obj;
    DataPack_Callback pack_callback;
    uint32_t proto_time;

    bool in_proto;
    bool lock_proto;
} SrvComProto_MsgInfo_TypeDef;

typedef struct
{
    bool init_state;
    SrvComProto_Type_List Proto_Type;
    // SrvComProto_Data_TypeDef proto_data;
} SrvComProto_Monitor_TypeDef;

typedef struct
{
    bool valid;
    SrvComProto_ProtoData_Type_List pac_type;

    uint8_t *p_buf;
    uint16_t size;

}SrvComProto_Msg_StreamIn_TypeDef;

typedef struct
{
    bool (*init)(SrvComProto_Type_List type, uint8_t *arg);
    SrvComProto_Type_List (*get_msg_type)(void);
    SrvComProto_Msg_StreamIn_TypeDef (*msg_decode)(uint8_t *p_data, uint16_t size);
    bool (*mav_msg_decode)(uint8_t *p_buf, uint32_t len);
    bool (*mav_msg_OutObj_init)(SrvComProto_MsgInfo_TypeDef *msg, SrvComProto_MavPackInfo_TypeDef pck_info, uint32_t period);
    bool (*mav_msg_enable_ctl)(SrvComProto_MsgInfo_TypeDef *msg, bool state);
    bool (*mav_msg_stream)(SrvComProto_MsgInfo_TypeDef *msg, SrvComProto_Stream_TypeDef *com_stream, void *arg, ComProto_Callback tx_cb);
} SrvComProto_TypeDef;

extern SrvComProto_TypeDef SrvComProto;

#endif
