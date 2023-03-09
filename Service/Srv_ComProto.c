#include "Srv_ComProto.h"
#include "Srv_Receiver.h"
#include "SrvMPU_Sample.h"
#include "DataPipe.h"
#include "mmu.h"
#include "runtime.h"

SrvComProto_Monitor_TypeDef SrvComProto_monitor = {0};

/* Pipe Object */
DataPipe_CreateDataObj(SrvIMU_UnionData_TypeDef, PtlPriIMU_Data);
DataPipe_CreateDataObj(SrvIMU_UnionData_TypeDef, PtlSecIMU_Data);

DataPipe_CreateDataObj(SrvRecever_RCSig_TypeDef, Proto_Rc);

/* internal function */
static uint16_t SrvComProto_MavMsg_Raw_IMU(SrvComProto_MsgInfo_TypeDef *pck);
static void SrvComProto_PipeRcTelemtryDataFinish_Callback(DataPipeObj_TypeDef *obj);

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

    /* only init one time */
    if (SrvComProto_monitor.init_state)
        return true;

    /* init pipe object */
    memset(DataPipe_DataObjAddr(PtlPriIMU_Data), NULL, DataPipe_DataSize(PtlPriIMU_Data));
    IMU_Ptl_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(PtlPriIMU_Data);
    IMU_Ptl_DataPipe.data_size = DataPipe_DataSize(PtlPriIMU_Data);
    IMU_Ptl_DataPipe.trans_finish_cb = SrvComProto_PipeRcTelemtryDataFinish_Callback;
    DataPipe_Set_RxInterval(&IMU_Ptl_DataPipe, Runtime_MsToUs(5)); /* limit pipe frequence to 200Hz */
    DataPipe_Enable(&IMU_Ptl_DataPipe);

    /* init pipe object */
    memset(DataPipe_DataObjAddr(Proto_Rc), 0, DataPipe_DataSize(Proto_Rc));
    Receiver_ptl_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Proto_Rc);
    Receiver_ptl_DataPipe.data_size = DataPipe_DataSize(Proto_Rc);
    Receiver_ptl_DataPipe.trans_finish_cb = SrvComProto_PipeRcTelemtryDataFinish_Callback;
    DataPipe_Set_RxInterval(&Receiver_ptl_DataPipe, Runtime_MsToUs(20)); /* limit pipe frequence to 50Hz */
    DataPipe_Enable(&Receiver_ptl_DataPipe);

    memset(&SrvComProto_monitor, 0, sizeof(SrvComProto_monitor));
    SrvComProto_monitor.Proto_Type = type;
    SrvComProto_monitor.init_state = true;

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

        com_stream->size = msg.pack_callback((uint8_t *)&msg);

        if (com_stream->size && ((com_stream->size + MAVLINK_NUM_NON_PAYLOAD_BYTES) <= com_stream->max_size))
        {
            com_stream->size = mavlink_msg_to_send_buffer(com_stream->p_buf, msg.msg_obj);

            tx_cb(com_stream->p_buf, com_stream->size);
        }

    quit_proto:
        msg.in_proto = false;
    }
}

static uint16_t SrvComProto_MavMsg_Raw_IMU(SrvComProto_MsgInfo_TypeDef *pck)
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

    return mavlink_msg_scaled_imu_pack_chan(pck->pck_info.system_id,
                                            pck->pck_info.component_id,
                                            pck->pck_info.chan, pck->msg_obj,
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

static void SrvComProto_PipeRcTelemtryDataFinish_Callback(DataPipeObj_TypeDef *obj)
{
    if (obj == NULL)
        return;

    if (obj == &Receiver_ptl_DataPipe)
    {
    }
    else if (obj == &IMU_Ptl_DataPipe)
    {
        //     SrvComProto_Fill_IMU(DataPipe_DataObj(PtlPriIMU_Data).data.time_stamp,
        //                          DataPipe_DataObj(PtlPriIMU_Data).data.acc[Axis_X],
        //                          DataPipe_DataObj(PtlPriIMU_Data).data.acc[Axis_Y],
        //                          DataPipe_DataObj(PtlPriIMU_Data).data.acc[Axis_Z],
        //                          DataPipe_DataObj(PtlPriIMU_Data).data.gyr[Axis_X],
        //                          DataPipe_DataObj(PtlPriIMU_Data).data.gyr[Axis_Y],
        //                          DataPipe_DataObj(PtlPriIMU_Data).data.gyr[Axis_Z]);
    }
}
