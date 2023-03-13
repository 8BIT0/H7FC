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
static bool Srv_ComProto_MsgObj_Init(SrvComProto_MsgInfo_TypeDef *msg, SrvComProto_MavPackInfo_TypeDef pck_info, uint32_t period);
static void SrvComProto_MsgToStream(SrvComProto_MsgInfo_TypeDef msg, SrvComProto_Stream_TypeDef *com_stream, ComProto_Callback tx_cb);
static bool SrvComProto_MsgEnable_Control(SrvComProto_MsgInfo_TypeDef *msg, bool state);
static SrvComProto_Type_List Srv_ComProto_GetType(void);

SrvComProto_TypeDef SrvComProto = {
    .init = Srv_ComProto_Init,
    .get_msg_type = Srv_ComProto_GetType,
    .mav_msg_stream = SrvComProto_MsgToStream,
    .mav_msg_enable_ctl = SrvComProto_MsgEnable_Control,
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

static bool Srv_ComProto_MsgObj_Init(SrvComProto_MsgInfo_TypeDef *msg, SrvComProto_MavPackInfo_TypeDef pck_info, uint32_t period)
{
    if ((msg == NULL) ||
        (period == 0))
        return false;

    msg->in_proto = false;
    msg->lock_proto = true;

    msg->pck_info = pck_info;
    msg->period = period;
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
    case MAV_CompoID_Attitude:
        break;

    case MAV_CompoID_RC_Channel:
        break;

    case MAV_CompoID_Raw_IMU:
        msg->pack_callback = SrvComProto_MavMsg_Raw_IMU;
        break;

    default:
        MMU_Free(msg->msg_obj);
        return false;
    }

    msg->lock_proto = false;

    return true;
}

static void SrvComProto_MsgToStream(SrvComProto_MsgInfo_TypeDef msg, SrvComProto_Stream_TypeDef *com_stream, ComProto_Callback tx_cb)
{
    if (msg.enable && com_stream && com_stream->p_buf && com_stream->size && msg.pack_callback)
    {
        msg.in_proto = true;

        if ((msg.proto_time) && (Get_CurrentRunningMs() - msg.proto_time < msg.period))
        {
            msg.in_proto = false;
            return;
        }

        com_stream->size = msg.pack_callback((uint8_t *)&msg);

        if (com_stream->size && ((com_stream->size + MAVLINK_NUM_NON_PAYLOAD_BYTES) <= com_stream->max_size))
        {
            com_stream->size = mavlink_msg_to_send_buffer(com_stream->p_buf, msg.msg_obj);

            if (tx_cb)
                tx_cb(com_stream->p_buf, com_stream->size);

            msg.proto_time = Get_CurrentRunningMs();
        }

        msg.in_proto = false;
    }
}

static bool SrvComProto_MsgEnable_Control(SrvComProto_MsgInfo_TypeDef *msg, bool state)
{
    if (msg == NULL)
        return false;

    msg->enable = state;
    return true;
}

static uint16_t SrvComProto_MavMsg_Raw_IMU(SrvComProto_MsgInfo_TypeDef *pck)
{
    uint32_t time_stamp = 0;
    float acc_scale = 0.0f;
    float gyr_scale = 0.0f;
    float acc_x = 0.0f;
    float acc_y = 0.0f;
    float acc_z = 0.0f;
    float gyr_x = 0.0f;
    float gyr_y = 0.0f;
    float gyr_z = 0.0f;
    float tmpr = 0.0f;

    SrvDataHub.get_raw_imu(&time_stamp, &acc_scale, &gyr_scale, &acc_x, &acc_y, &acc_z, &gyr_x, &gyr_y, &gyr_z, &tmpr);

    int16_t i_acc_x = (int16_t)(acc_x * acc_scale);
    int16_t i_acc_y = (int16_t)(acc_y * acc_scale);
    int16_t i_acc_z = (int16_t)(acc_z * acc_scale);

    int16_t i_gyr_x = (int16_t)(gyr_x * gyr_scale);
    int16_t i_gyr_y = (int16_t)(gyr_y * gyr_scale);
    int16_t i_gyr_z = (int16_t)(gyr_z * gyr_scale);

    /* we dont have any mag sensor currently */
    return mavlink_msg_scaled_imu_pack_chan(pck->pck_info.system_id,
                                            pck->pck_info.component_id,
                                            pck->pck_info.chan, pck->msg_obj,
                                            time_stamp,
                                            i_acc_x, i_acc_y, i_acc_z,
                                            i_gyr_x, i_gyr_y, i_gyr_z,
                                            0, 0, 0);
}
