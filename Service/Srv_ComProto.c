#include "Srv_ComProto.h"
#include "Srv_DataHub.h"
#include "DataPipe.h"
#include "mmu.h"
#include "runtime.h"

/* only can use one hardware port at one time */
/* still can be optmize / use multi port proto mavlink frame */

SrvComProto_Monitor_TypeDef SrvComProto_monitor = {
    .Proto_Type = SrvComProto_Type_None,
    .init_state = false,
};

/* internal function */
static uint16_t SrvComProto_MavMsg_Raw_IMU(SrvComProto_MsgInfo_TypeDef *pck);

/* external function */
static bool Srv_ComProto_Init(SrvComProto_Type_List type, uint8_t *arg);
static bool Srv_ComProto_MsgObj_Init(SrvComProto_MsgInfo_TypeDef *msg, SrvComProto_MavPackInfo_TypeDef pck_info,
                                     uint32_t period, SrvComProto_IOType_List io_dir);
static void SrvComProto_MsgToStream(SrvComProto_MsgInfo_TypeDef msg, SrvComProto_Stream_TypeDef *com_stream);
static SrvComProto_Type_List Srv_ComProto_GetType(void);

SrvComProto_TypeDef SrvComProto = {
    .init = Srv_ComProto_Init,
    .get_msg_type = Srv_ComProto_GetType,
    .mav_msg_stream = SrvComProto_MsgToStream,
};

static bool Srv_ComProto_Init(SrvComProto_Type_List type, uint8_t *arg)
{
    UNUSED(arg);

    /* only init one time */
    if (SrvComProto_monitor.init_state)
        return true;

    SrvDataHub.init();

    memset(&SrvComProto_monitor, 0, sizeof(SrvComProto_monitor));
    SrvComProto_monitor.Proto_Type = type;
    SrvComProto_monitor.init_state = true;

    return true;
}

static SrvComProto_Type_List Srv_ComProto_GetType(void)
{
    return SrvComProto_monitor.Proto_Type;
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

static void SrvComProto_MsgToStream(SrvComProto_MsgInfo_TypeDef msg, SrvComProto_Stream_TypeDef *com_stream)
{
    if (com_stream && com_stream->p_buf && com_stream->size && IS_PROTO_OUT(msg.io_type))
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

        com_stream->size = msg.pack_callback((uint8_t *)&msg);

        if (com_stream->size && ((com_stream->size + MAVLINK_NUM_NON_PAYLOAD_BYTES) <= com_stream->max_size))
        {
            com_stream->size = mavlink_msg_to_send_buffer(com_stream->p_buf, msg.msg_obj);
        }

    quit_proto:
        msg.in_proto = false;
    }
}

static uint16_t SrvComProto_MavMsg_Raw_IMU(SrvComProto_MsgInfo_TypeDef *pck)
{
    // int16_t acc_x = SrvComProto_monitor.proto_data.acc_x * SrvComProto_monitor.proto_data.acc_scale;
    // int16_t acc_y = SrvComProto_monitor.proto_data.acc_y * SrvComProto_monitor.proto_data.acc_scale;
    // int16_t acc_z = SrvComProto_monitor.proto_data.acc_z * SrvComProto_monitor.proto_data.acc_scale;

    // int16_t gyr_x = SrvComProto_monitor.proto_data.gyr_x * SrvComProto_monitor.proto_data.gyr_scale;
    // int16_t gyr_y = SrvComProto_monitor.proto_data.gyr_y * SrvComProto_monitor.proto_data.gyr_scale;
    // int16_t gyr_z = SrvComProto_monitor.proto_data.gyr_z * SrvComProto_monitor.proto_data.gyr_scale;

    // int16_t mag_x = SrvComProto_monitor.proto_data.mag_x * SrvComProto_monitor.proto_data.mag_scale;
    // int16_t mag_y = SrvComProto_monitor.proto_data.mag_y * SrvComProto_monitor.proto_data.mag_scale;
    // int16_t mag_z = SrvComProto_monitor.proto_data.mag_z * SrvComProto_monitor.proto_data.mag_scale;

    // return mavlink_msg_scaled_imu_pack_chan(pck->pck_info.system_id,
    //                                         pck->pck_info.component_id,
    //                                         pck->pck_info.chan, pck->msg_obj,
    //                                         SrvComProto_monitor.proto_data.imu_update_time,
    //                                         acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, mag_x, mag_y, mag_z);
}
