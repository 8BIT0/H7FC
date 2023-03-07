#include "Srv_ComProto.h"
#include "mmu.h"

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
                                     uint32_t period, SrvComProto_IOType_List io_dir,
                                     SrvComProto_Stream_TypeDef tar_stream);
static void SrvComProto_Msg(SrvComProto_MsgInfo_TypeDef msg, SrvComProto_Stream_TypeDef *com_stream, ComProto_Callback tx_cb);


SrvComProto_TypeDef SrvComProto = {
    .init = Srv_ComProto_Init,
    .mav_msg_proto = SrvComProto_Msg,
};

static bool Srv_ComProto_Init(SrvComProto_Type_List type, uint8_t *arg)
{
    UNUSED(arg);

    memset(&SrvComProto_monitor, 0, sizeof(SrvComProto_monitor));
    SrvComProto_monitor.Proto_Type = type;

    return true;
}

static bool Srv_ComProto_MsgObj_Init(SrvComProto_MsgInfo_TypeDef *msg, SrvComProto_MavPackInfo_TypeDef pck_info,
                                     uint32_t period, SrvComProto_IOType_List io_dir,
                                     SrvComProto_Stream_TypeDef tar_stream)
{
    if ((msg == NULL) ||
        (period == 0) ||
        (tar_stream.p_buf == NULL) ||
        (tar_stream.size == 0))
        return false;

    msg->in_proto = false;
    msg->lock_proto = true;

    msg->pck_info = pck_info;
    msg->period = period;
    msg->io_type = io_dir;
    msg->tar_obj = tar_stream;
    msg->proto_time = 0;

    /* create mavlink message object */
    msg->msg_obj = (mavlink_message_t *)MMU_Malloc(sizeof(mavlink_message_t));

    if (msg->msg_obj)
    {
        MMU_Free(msg->msg_ob);
        return false;
    }

    memset(msg->msg_obj, 0, sizeof(mavlink_message_t));

    /* set mavlink data structure value set function */
    switch((uint8_t)pck_info.component_id)
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
            MMU_Free(msg->msg_ob);
            return false;
    }

    msg->lock_proto = false;

    return true;
}

static void SrvComProto_Msg(SrvComProto_MsgInfo_TypeDef msg, SrvComProto_Stream_TypeDef *com_stream, ComProto_Callback tx_cb)
{

}

static void SrvComProto_MavMsg_Raw_IMU(SrvComProto_MsgInfo_TypeDef pck)
{
    uint16_t OutStream_Len = mavlink_msg_scaled_imu_pack_chan( pck.pck_info.system_id, pck.pck_info.component_id, pck.pck_info.chan, pck.msg_obj,
                                                               time_boot_ms, xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag);
}

static void SrvComProto_MavMsg_Raw_IMU2(SrvComProto_MsgInfo_TypeDef pck)
{
}

static void SrvComProto_MavMsg_Scale_IMU(SrvComProto_MsgInfo_TypeDef pck)
{
}

static void SrvComProto_MavMsg_Scale_IMU2(SrvComProto_MsgInfo_TypeDef pck)
{
}

static void SrvComProto_MavMsg_Raw_RcChannel(SrvComProto_MsgInfo_TypeDef pck)
{
}
