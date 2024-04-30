#ifndef __SRV_COMPROTO_H
#define __SRV_COMPROTO_H

#include "Srv_OsCommon.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "../MAVLink/common/mavlink.h"

typedef bool (*ComProto_Callback)(void *arg, uint8_t *p_data, uint32_t len);
typedef uint16_t (*DataPack_Callback)(uint8_t *pck);
typedef bool (*SrvComProto_MavMsgIn_Callback)(void const *mav_data, void *cus_data);
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
    MAV_SysID_OPC,   /* On plane computer data input */
    MAV_SysID_Radio,
} SrvComProto_SysID_List;

typedef enum
{
    MAV_ParamOperation_Read = 0,
    MAV_ParamOperation_Write,
    MAV_ParamOperation_Clear,
    MAV_ParamOperation_ToDefault,
} SrvComProto_ParaOperationType_List;

typedef enum
{
    /* output from drone */
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

    /* input to drone from on plane computer */
    MAV_CompoID_Ctl_Gyro,
    MAV_CompoID_Ctl_Attitude,
    MAV_CompoID_Ctl_Altitude,
    MAV_CompoID_Ctl_RC_Channel,
    MAV_CompoID_Ctl_MotoCtl,
    MAV_CompoID_Ctl_FileAdapter,

    MAV_CompoID_Ctl_ParaOperation,
    MAV_CompoID_Ctl_PIDPara_GyrX,
    MAV_CompoID_Ctl_PIDPara_GyrY,
    MAV_CompoID_Ctl_PIDPara_GyrZ,
    MAV_CompoID_Ctl_PIDPara_Roll,
    MAV_CompoID_Ctl_PIDPata_Pitch,
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
} SrvComProto_Msg_StreamIn_TypeDef;

typedef enum
{
    MavIn_Msg_Gyr = 0,
    MavIn_Msg_Att,
    MavIn_Msg_Alt,
    MavIn_Msg_RC,
    MavIn_Msg_MotoCtl,
    MavIn_Msg_FileAdapter,
    MavIn_Msg_PID_Para_GyrX,
    MavIn_Msg_PID_Para_GyrY,
    MavIn_Msg_PID_Para_GyrZ,
    MavIn_Msg_PID_Para_Roll,
    MavIn_Msg_PID_Para_Pitch,
    MavIn_Msg_PID_Para_Operation,
} SrvComProto_MavInMsgType_List;

typedef struct
{
    uint32_t port_addr;

    void *cus_p_gyr;
    void *cus_p_att;
    void *cus_p_alt;
    void *cus_p_RC;
    void *cus_p_moto;
    void *cus_p_FileAdapter;
    void *cus_p_pidpara_gyrox;
    void *cus_p_pidpara_gyroy;
    void *cus_p_pidpara_gyroz;
    void *cus_p_pidpara_roll;
    void *cus_p_pidpara_pitch;
    void *cus_p_paraoperation;

    uint32_t Gyr_Cnt;
    uint32_t Att_Cnt;
    uint32_t Alt_Cnt;
    uint32_t RC_Cnt;
    uint32_t Moto_Cnt;
    uint32_t FileAdapter_Cnt;
    uint32_t PID_Para_GyrX_Cnt;
    uint32_t PID_Para_GyrY_Cnt;
    uint32_t PID_Para_GyrZ_Cnt;
    uint32_t PID_Para_Roll_Cnt;
    uint32_t PID_Para_Pitch_Cnt;
    uint32_t Para_Operation_Cnt;

    SrvComProto_MavMsgIn_Callback MavMsg_Gyr_Callback;
    SrvComProto_MavMsgIn_Callback MavMsg_Att_Callback;
    SrvComProto_MavMsgIn_Callback MavMsg_Alt_Callback;
    SrvComProto_MavMsgIn_Callback MavMsg_RC_Callback;
    SrvComProto_MavMsgIn_Callback MavMsg_MotoCtl_Callback;
    SrvComProto_MavMsgIn_Callback MavMsg_FileAdapter_Callback;
    SrvComProto_MavMsgIn_Callback MavMsg_PIDPara_GyroX_Callback;
    SrvComProto_MavMsgIn_Callback MavMsg_PIDPara_GyroY_Callback;
    SrvComProto_MavMsgIn_Callback MavMsg_PIDPara_GyroZ_Callback;
    SrvComProto_MavMsgIn_Callback MavMsg_PIDPara_Roll_Callback;
    SrvComProto_MavMsgIn_Callback MavMsg_PIDPara_Pitch_Callback;
    SrvComProto_MavMsgIn_Callback MavMsg_ParamOperation_Callback;
} SrvComProto_MsgObj_TypeDef;

typedef struct
{
    bool (*init)(SrvComProto_Type_List type, uint8_t *arg);
    SrvComProto_Type_List (*get_msg_type)(void);
    SrvComProto_Msg_StreamIn_TypeDef (*msg_decode)( SrvComProto_MsgObj_TypeDef *obj, uint8_t *p_data, uint16_t size);
    bool (*mav_msg_obj_init)(SrvComProto_MsgInfo_TypeDef *msg, SrvComProto_MavPackInfo_TypeDef pck_info, uint32_t period);
    bool (*mav_msg_enable_ctl)(SrvComProto_MsgInfo_TypeDef *msg, bool state);
    bool (*mav_msg_stream)(SrvComProto_MsgInfo_TypeDef *msg, SrvComProto_Stream_TypeDef *com_stream, void *arg, ComProto_Callback tx_cb);
    void (*mav_msg_set_input_callback)(SrvComProto_MsgObj_TypeDef *obj, SrvComProto_MavInMsgType_List type, SrvComProto_MavMsgIn_Callback callback, void *p_cus_data);
} SrvComProto_TypeDef;

extern SrvComProto_TypeDef SrvComProto;

#endif
