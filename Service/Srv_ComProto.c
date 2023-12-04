#include "Srv_ComProto.h"
#include "Srv_DataHub.h"
#include "DataPipe.h"
#include "Srv_OsCommon.h"
#include "Bsp_Uart.h"

/* only can use one hardware port at one time */
/* still can be optmize / use multi port proto mavlink frame */

SrvComProto_Monitor_TypeDef SrvComProto_monitor = {
    .Proto_Type = SrvComProto_Type_None,
    .init_state = false,
};

/* internal function */
static uint16_t SrvComProto_MavMsg_Raw_IMU(SrvComProto_MsgInfo_TypeDef *pck);
static uint16_t SrvComProto_MavMsg_Scaled_IMU(SrvComProto_MsgInfo_TypeDef *pck);
static uint16_t SrvComProto_MavMsg_Attitude(SrvComProto_MsgInfo_TypeDef *pck);
static uint16_t SrvConProto_MavMsg_RC(SrvComProto_MsgInfo_TypeDef *pck);

/* external function */
static bool Srv_ComProto_Init(SrvComProto_Type_List type, uint8_t *arg);
static bool Srv_ComProto_MsgObj_Init(SrvComProto_MsgInfo_TypeDef *msg, SrvComProto_MavPackInfo_TypeDef pck_info, uint32_t period);
static void SrvComProto_MsgToStream(SrvComProto_MsgInfo_TypeDef *msg, SrvComProto_Stream_TypeDef *com_stream, void *arg, ComProto_Callback tx_cb);
static bool SrvComProto_MsgEnable_Control(SrvComProto_MsgInfo_TypeDef *msg, bool state);
static SrvComProto_Type_List Srv_ComProto_GetType(void);
static SrvComProto_Msg_StreamIn_TypeDef SrvComProto_MavMsg_Input_Decode(uint8_t *p_data, uint16_t size);

SrvComProto_TypeDef SrvComProto = {
    .init = Srv_ComProto_Init,
    .mav_msg_obj_init = Srv_ComProto_MsgObj_Init,
    .get_msg_type = Srv_ComProto_GetType,
    .mav_msg_stream = SrvComProto_MsgToStream,
    .mav_msg_enable_ctl = SrvComProto_MsgEnable_Control,
    .msg_decode = SrvComProto_MavMsg_Input_Decode,
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
    msg->msg_obj = (mavlink_message_t *)SrvOsCommon.malloc(sizeof(mavlink_message_t));

    if (msg->msg_obj == NULL)
    {
        SrvOsCommon.free(msg->msg_obj);
        return false;
    }

    memset(msg->msg_obj, 0, sizeof(mavlink_message_t));

    /* set mavlink data structure value set function */
    switch ((uint8_t)pck_info.component_id)
    {
    case MAV_CompoID_Attitude:
        msg->pack_callback = SrvComProto_MavMsg_Attitude;
        break;

    case MAV_CompoID_RC_Channel:
        msg->pack_callback = SrvConProto_MavMsg_RC;
        break;

    case MAV_CompoID_Raw_IMU:
        msg->pack_callback = SrvComProto_MavMsg_Raw_IMU;
        break;

    case MAV_CompoID_Scaled_IMU:
        msg->pack_callback = SrvComProto_MavMsg_Scaled_IMU;
        break;

    case MAV_CompoID_MotoCtl:
        break;

    case MAV_CompoID_ServoCtl:
        break;

    default:
        SrvOsCommon.free(msg->msg_obj);
        msg->lock_proto = false;
        return false;
    }

    msg->lock_proto = false;

    return true;
}

static void SrvComProto_MsgToStream(SrvComProto_MsgInfo_TypeDef *msg, SrvComProto_Stream_TypeDef *com_stream, void *arg, ComProto_Callback tx_cb)
{
    if (msg->enable && com_stream && com_stream->p_buf && msg->pack_callback)
    {
        msg->in_proto = true;

        if ((msg->proto_time) && (SrvOsCommon.get_os_ms() - msg->proto_time < msg->period))
        {
            msg->in_proto = false;
            return;
        }

        com_stream->size = msg->pack_callback((uint8_t *)msg);

        if (com_stream->size && ((com_stream->size + MAVLINK_NUM_NON_PAYLOAD_BYTES) <= com_stream->max_size))
        {
            com_stream->size = mavlink_msg_to_send_buffer(com_stream->p_buf, msg->msg_obj);

            if (tx_cb)
            {
                msg->proto_cnt ++;
                tx_cb(arg, com_stream->p_buf, com_stream->size);
            }

            msg->proto_time = SrvOsCommon.get_os_ms();
        }

        msg->in_proto = false;
    }
}

static bool SrvComProto_MsgEnable_Control(SrvComProto_MsgInfo_TypeDef *msg, bool state)
{
    if (msg == NULL)
        return false;

    msg->enable = state;
    return true;
}

static uint16_t SrvComProto_MavMsg_Moto(SrvComProto_MsgInfo_TypeDef *pck)
{
    uint32_t time_stamp = 0;
    uint16_t moto[8] = {0};
    uint8_t moto_dir[8] = {0};
    uint8_t moto_cnt = 0;

    SrvDataHub.get_moto(&time_stamp, &moto_cnt, moto, moto_dir);

    return mavlink_msg_servo_output_raw_pack_chan(pck->pck_info.system_id,
                                                  pck->pck_info.component_id,
                                                  pck->pck_info.chan, pck->msg_obj,
                                                  time_stamp, MAV_ActuatorPort_Moto,
                                                  moto[0], moto[1], moto[2], moto[3],
                                                  moto[4], moto[5], moto[6], moto[7]);
}

static uint16_t SrvComProto_MavMsg_MotoDir(SrvComProto_MsgInfo_TypeDef *pck)
{
    uint32_t time_stamp = 0;
    uint16_t moto[8] = {0};
    uint8_t moto_dir[8] = {0};
    uint8_t moto_cnt = 0;

    SrvDataHub.get_moto(&time_stamp, &moto_cnt, moto, moto_dir);

    return mavlink_msg_servo_output_raw_pack_chan(pck->pck_info.system_id,
                                                  pck->pck_info.component_id,
                                                  pck->pck_info.chan, pck->msg_obj,
                                                  time_stamp, MAV_ActuatorPort_MotoDir,
                                                  moto_dir[0], moto_dir[1], moto_dir[2], moto_dir[3],
                                                  moto_dir[4], moto_dir[5], moto_dir[6], moto_dir[7]);
}

static uint16_t SrvComProto_MavMsg_Servo(SrvComProto_MsgInfo_TypeDef *pck)
{
    uint32_t time_stamp = 0;
    uint16_t servo[8] = {0};
    uint8_t servo_dir[8] = {0};
    uint8_t servo_cnt = 0;

    SrvDataHub.get_servo(&time_stamp, &servo_cnt, servo, servo_dir);

    return mavlink_msg_servo_output_raw_pack_chan(pck->pck_info.system_id,
                                                  pck->pck_info.component_id,
                                                  pck->pck_info.chan, pck->msg_obj,
                                                  time_stamp, MAV_ActuatorPort_Servo,
                                                  servo[0], servo[1], servo[2], servo[3],
                                                  servo[4], servo[5], servo[6], servo[7]);
}

static uint16_t SrvComProto_MavMsg_ServoDir(SrvComProto_MsgInfo_TypeDef *pck)
{
    uint32_t time_stamp = 0;
    uint16_t servo[8] = {0};
    uint8_t servo_dir[8] = {0};
    uint8_t servo_cnt = 0;

    SrvDataHub.get_servo(&time_stamp, &servo_cnt, servo, servo_dir);

    return mavlink_msg_servo_output_raw_pack_chan(pck->pck_info.system_id,
                                                  pck->pck_info.component_id,
                                                  pck->pck_info.chan, pck->msg_obj,
                                                  time_stamp, MAV_ActuatorPort_ServoDir,
                                                  servo_dir[0], servo_dir[1], servo_dir[2], servo_dir[3],
                                                  servo_dir[4], servo_dir[5], servo_dir[6], servo_dir[7]);
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
    uint8_t imu_err_code = 0;

    SrvDataHub.get_raw_imu(&time_stamp, &acc_scale, &gyr_scale, &acc_x, &acc_y, &acc_z, &gyr_x, &gyr_y, &gyr_z, &tmpr, &imu_err_code);

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

static uint16_t SrvComProto_MavMsg_Scaled_IMU(SrvComProto_MsgInfo_TypeDef *pck)
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
    uint8_t imu_err_code = 0;

    SrvDataHub.get_scaled_imu(&time_stamp, &acc_scale, &gyr_scale, &acc_x, &acc_y, &acc_z, &gyr_x, &gyr_y, &gyr_z, &tmpr, &imu_err_code);

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

static uint16_t SrvComProto_MavMsg_Attitude(SrvComProto_MsgInfo_TypeDef *pck)
{
    uint32_t time_stamp = 0;
    
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;
    
    float q0 = 0.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;

    SrvDataHub.get_attitude(&time_stamp, &pitch, &roll, &yaw, &q0, &q1, &q2, &q3);

    return mavlink_msg_attitude_pack_chan(pck->pck_info.system_id,
                                          pck->pck_info.component_id,
                                          pck->pck_info.chan, pck->msg_obj,
                                          time_stamp,
                                          roll, pitch, yaw, 
                                          0.0f, 0.0f, 0.0f);
}

static uint16_t SrvConProto_MavMsg_RC(SrvComProto_MsgInfo_TypeDef *pck)
{
    uint32_t time_stamp = 0;
    uint8_t channel_num = 0;
    uint8_t rssi = 0;
    uint16_t channel[32] = {0};

    SrvDataHub.get_rc(&time_stamp, channel, &channel_num);

    return mavlink_msg_rc_channels_pack_chan(pck->pck_info.system_id,
                                             pck->pck_info.component_id,
                                             pck->pck_info.chan, pck->msg_obj,
                                             time_stamp, channel_num,
                                             channel[0],  channel[1],  channel[2],  channel[3],
                                             channel[4],  channel[5],  channel[6],  channel[7],
                                             channel[8],  channel[9],  channel[10], channel[11],
                                             channel[12], channel[13], channel[14], channel[15],
                                             channel[16], channel[17], rssi);
}

static uint16_t SrvComProto_MavMsg_Altitude(SrvComProto_MsgInfo_TypeDef *pck)
{
    uint32_t time_stamp = 0;
    uint8_t error = 0;
    float baro_alt = 0.0f;
    float baro_alt_offset = 0.0f;
    float baro_tempra = 0.0f;

    SrvDataHub.get_baro_altitude(time_stamp, &baro_alt, &baro_alt_offset, &baro_tempra, &error);

    return mavlink_msg_altitude_pack_chan(pck->pck_info.system_id,
                                          pck->pck_info.component_id,
                                          pck->pck_info.chan, pck->msg_obj,
                                          time_stamp,
                                          baro_alt, 0, 0, 0, 0, 0);
}

static SrvComProto_Msg_StreamIn_TypeDef SrvComProto_MavMsg_Input_Decode(uint8_t *p_data, uint16_t size)
{
    SrvComProto_Msg_StreamIn_TypeDef stream_in;
    uint16_t payload_size = 0;
    uint8_t default_channel = 0;
    mavlink_message_t mav_msg;
    mavlink_status_t mav_sta;
    uint8_t mav_decode = 0;

    memset(&stream_in, 0, sizeof(SrvComProto_Msg_StreamIn_TypeDef));

    /* match cli */
    if((p_data[size - 1] == '\n') && (p_data[size - 2] == '\r'))
    {
        stream_in.pac_type = ComFrame_CLI;
        stream_in.valid = true;
        stream_in.size = size;
        stream_in.p_buf = p_data;
    
        goto input_stream_valid;
    }
    
    memset(&mav_msg, 0, sizeof(mavlink_message_t));
    memset(&mav_sta, 0, sizeof(mavlink_status_t));
    
    /* match mavlink message */
    for(uint16_t i = 0; i < size; i++)
    {
        mav_decode = mavlink_frame_char(default_channel, p_data[i], &mav_msg, &mav_sta);

        if(mav_decode && (i < size))
        {
            /* get multi protocol frame in one transmition and first protocol frame is mavlink message */
            /* only decode first pack */
            break;
        }
    }

    if(mav_decode)
    {
        stream_in.pac_type = ComFrame_MavMsg;
        stream_in.valid = true;
        stream_in.size = size;
        stream_in.p_buf = p_data;
    }

    /* custom frame input check */

input_stream_valid:
    return stream_in;
}
