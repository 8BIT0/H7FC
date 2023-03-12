#ifndef __SRV_COMPROTO_H
#define __SRV_COMPROTO_H

#include "../System/runtime/runtime.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "../MAVLink/common/mavlink.h"

typedef bool (*ComProto_Callback)(uint8_t *p_data, uint32_t len);
typedef uint16_t (*DataPack_Callback)(uint8_t *pck);

typedef enum
{
    SysID_Drone = 1,
    SysID_ROS,
} SrvComProto_SysID_List;

typedef enum
{
    CompoID_Raw_IMU = 1,
    CompoID_Scaled_IMU,
    CompoID_Raw_Mag,
    CompoID_Scaled_Mag,
    CompoID_RC_Channel,
    CompoID_MotoCtl,
    CompoID_ServoCtl,
} SrvComProto_ComponentID_List;

typedef enum
{
    SrvComProto_Type_None = 0,
    SrvComProto_Type_Cus,
    SrvComProto_Type_MAV,
} SrvComProto_Type_List;

typedef enum
{
    MAV_Component_Attitude = 1,
    MAV_Component_Rc_Channel,
    MAV_Component_Raw_IMU,
    MAV_Component_Raw_IMU2,
} SrvComProto_MAVComponent_List;

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
    uint16_t period;

    SrvComProto_MavPackInfo_TypeDef pck_info;
    mavlink_message_t *msg_obj;
    DataPack_Callback pack_callback;
    SYSTEM_RunTime proto_time;

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
    void (*init)(SrvComProto_Type_List type, uint8_t *arg);
    SrvComProto_Type_List (*get_msg_type)(void);
    // void (*set_decode_callback)();

    bool (*mav_msg_decode)(uint8_t *p_buf, uint32_t len);
    bool (*mav_msg_obj_init)(SrvComProto_MsgInfo_TypeDef *msg, SrvComProto_MavPackInfo_TypeDef pck_info,
                             uint32_t period, SrvComProto_Stream_TypeDef tar_stream);
    bool (*mav_msg_stream)(SrvComProto_MsgInfo_TypeDef msg,
                           SrvComProto_Stream_TypeDef *com_stream);
} SrvComProto_TypeDef;

extern SrvComProto_TypeDef SrvComProto;

#endif
