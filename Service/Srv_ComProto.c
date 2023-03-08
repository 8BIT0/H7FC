#include "Srv_ComProto.h"
#include "DataPipe.h"
#include "mmu.h"
#include "runtime.h"

SrvComProto_Monitor_TypeDef SrvComProto_monitor;

/* internal function */
static void SrvComProto_MavMsg_Raw_IMU(SrvComProto_MsgInfo_TypeDef pck);
static void SrvComProto_MavMsg_Raw_IMU2(SrvComProto_MsgInfo_TypeDef pck);
static void SrvComProto_MavMsg_Scale_IMU(SrvComProto_MsgInfo_TypeDef pck);
static void SrvComProto_MavMsg_Scale_IMU2(SrvComProto_MsgInfo_TypeDef pck);
static void SrvComProto_MavMsg_Raw_RcChannel(SrvComProto_MsgInfo_TypeDef pck);

/* external function */
static bool Srv_ComProto_Init(SrvComProto_Type_List type, uint8_t *arg);
static bool Srv_ComProto_MsgObj_Init(SrvComProto_MsgInfo_TypeDef *msg, SrvComProto_MavPackInfo_TypeDef pck_info,
                                     uint32_t period, SrvComProto_IOType_List io_dir);
static void SrvComProto_Msg(SrvComProto_MsgInfo_TypeDef msg, SrvComProto_Stream_TypeDef *com_stream, ComProto_Callback tx_cb);

static void SrvComProto_Fill_IMU(uint32_t update_time, float acc_scale, float gyr_scale, float accx, float accy, float accz, float gyrx, float gyry, float gyrz);

SrvComProto_TypeDef SrvComProto = {
    .init = Srv_ComProto_Init,
    .mav_msg_proto = SrvComProto_Msg,

    .fill_imu = SrvComProto_Fill_IMU,
    .fill_mag = NULL,
    .fill_baro = NULL,
    .fill_tof = NULL,
    .fill_sonar = NULL,
    .fill_attitude = NULL,
};

static bool Srv_ComProto_Init(SrvComProto_Type_List type, uint8_t *arg)
{
    UNUSED(arg);

    memset(&SrvComProto_monitor, 0, sizeof(SrvComProto_monitor));
    SrvComProto_monitor.Proto_Type = type;

    return true;
}

static bool Srv_ComProto_MsgObj_Init(SrvComProto_MsgInfo_TypeDef *msg, SrvComProto_MavPackInfo_TypeDef pck_info,
                                     uint32_t period, SrvComProto_IOType_List io_dir)
{
    if ((msg == NULL) ||
        (period == 0))
        return false;

    msg->in_proto = false;
    msg->lock_proto = true;

    msg->pck_info = pck_info;
    msg->period = period;
    msg->io_type = io_dir;
    msg->proto_time = 0;

    /* create mavlink message object */
    msg->msg_obj = (mavlink_message_t *)MMU_Malloc(sizeof(mavlink_message_t));

    if (msg->msg_obj)
    {
        MMU_Free(msg->msg_obj);
        return false;
    }

    memset(msg->msg_obj, 0, sizeof(mavlink_message_t));

    /* set mavlink data structure value set function */
    switch ((uint8_t)pck_info.component_id)
    {
    case MAV_Component_Attitude:
        break;

    case MAV_Component_Rc_Channel:
        break;

    case MAV_Component_Raw_IMU:
        msg->pack_callback = SrvComProto_MavMsg_Raw_IMU;
        break;

    case MAV_Component_Raw_IMU2:
        break;

    default:
        MMU_Free(msg->msg_obj);
        return false;
    }

    msg->lock_proto = false;

    return true;
}

static void SrvComProto_Msg(SrvComProto_MsgInfo_TypeDef msg, SrvComProto_Stream_TypeDef *com_stream, ComProto_Callback tx_cb)
{
    if (com_stream && com_stream->p_buf && com_stream->size && tx_cb && IS_PROTO_OUT(msg.io_type))
    {
        msg.in_proto = true;

        if (msg.proto_time == 0)
        {
            msg.proto_time = Get_CurrentRunningMs();
        }
        else if (Get_CurrentRunningMs() - msg.proto_time < msg.period)
        {
            goto quit_proto;
        }

    quit_proto:
        msg.in_proto = false;
    }
}

static void SrvComProto_MavMsg_Raw_IMU(SrvComProto_MsgInfo_TypeDef pck)
{
    int16_t acc_x = SrvComProto_monitor.proto_data.acc_x * SrvComProto_monitor.proto_data.acc_scale;
    int16_t acc_y = SrvComProto_monitor.proto_data.acc_y * SrvComProto_monitor.proto_data.acc_scale;
    int16_t acc_z = SrvComProto_monitor.proto_data.acc_z * SrvComProto_monitor.proto_data.acc_scale;

    int16_t gyr_x = SrvComProto_monitor.proto_data.gyr_x * SrvComProto_monitor.proto_data.gyr_scale;
    int16_t gyr_y = SrvComProto_monitor.proto_data.gyr_y * SrvComProto_monitor.proto_data.gyr_scale;
    int16_t gyr_z = SrvComProto_monitor.proto_data.gyr_z * SrvComProto_monitor.proto_data.gyr_scale;

    int16_t mag_x = SrvComProto_monitor.proto_data.mag_x * SrvComProto_monitor.proto_data.mag_scale;
    int16_t mag_y = SrvComProto_monitor.proto_data.mag_y * SrvComProto_monitor.proto_data.mag_scale;
    int16_t mag_z = SrvComProto_monitor.proto_data.mag_z * SrvComProto_monitor.proto_data.mag_scale;

    uint16_t OutStream_Len = mavlink_msg_scaled_imu_pack_chan(pck.pck_info.system_id,
                                                              pck.pck_info.component_id,
                                                              pck.pck_info.chan, pck.msg_obj,
                                                              SrvComProto_monitor.proto_data.imu_update_time,
                                                              acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, mag_x, mag_y, mag_z);
}

static void SrvComProto_Fill_IMU(uint32_t update_time, float acc_scale, float gyr_scale, float accx, float accy, float accz, float gyrx, float gyry, float gyrz)
{
    SrvComProto_monitor.proto_data.imu_update_time = update_time;

    SrvComProto_monitor.proto_data.acc_scale = acc_scale;
    SrvComProto_monitor.proto_data.acc_x = accx;
    SrvComProto_monitor.proto_data.acc_y = accy;
    SrvComProto_monitor.proto_data.acc_z = accz;

    SrvComProto_monitor.proto_data.gyr_scale = gyr_scale;
    SrvComProto_monitor.proto_data.gyr_x = gyrx;
    SrvComProto_monitor.proto_data.gyr_y = gyry;
    SrvComProto_monitor.proto_data.gyr_z = gyrz;
}