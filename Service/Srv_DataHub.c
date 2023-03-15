#include "Srv_DataHub.h"

/* internal variable */
SrvDataHub_Monitor_TypeDef SrvDataHub_Monitor = {
    .init_state = false,
};

/* Pipe Object */
DataPipe_CreateDataObj(SrvIMU_UnionData_TypeDef, PtlPriIMU_Data);
DataPipe_CreateDataObj(SrvIMU_UnionData_TypeDef, PtlSecIMU_Data);

DataPipe_CreateDataObj(SrvRecever_RCSig_TypeDef, Proto_Rc);

/* internal function */
static void SrvComProto_PipeRcTelemtryDataFinish_Callback(DataPipeObj_TypeDef *obj);

/* external function */
static void SrvDataHub_Init(void);
static bool SrvDataHub_Get_Raw_IMU(uint32_t *time_stamp, float *acc_scale, float *gyr_scale, float *acc_x, float *acc_y, float *acc_z, float *gyr_x, float *gyr_y, float *gyr_z, float *tmpr);
static bool SrvDataHub_Get_Scaled_IMU(uint32_t *time_stamp, float *acc_scale, float *gyr_scale, float *acc_x, float *acc_y, float *acc_z, float *gyr_x, float *gyr_y, float *gyr_z, float *tmpr);
static bool SrvDataHub_Get_Raw_Mag(uint32_t *time_stamp, float *scale, float *mag_x, float *mag_y, float *mag_z);
static bool SrvDataHub_Get_Scaled_Mag(uint32_t *time_stamp, float *scale, float *mag_x, float *mag_y, float *mag_z);
static bool SrvDataHub_Get_Arm(bool *arm);
static bool SrvDataHub_Get_Failsafe(bool *failsafe);
static bool SrvDataHub_Get_ControlMode(uint8_t *mode);
static bool SrvDataHub_Get_RcChannel(uint32_t *time_stamp, uint16_t *ch, uint8_t *ch_cum);

/* external variable */
SrvDataHub_TypeDef SrvDataHub = {
    .init = SrvDataHub_Init,
    .get_raw_imu = SrvDataHub_Get_Raw_IMU,
    .get_scaled_imu = SrvDataHub_Get_Scaled_IMU,
    .get_raw_mag = SrvDataHub_Get_Raw_Mag,
    .get_scaled_mag = SrvDataHub_Get_Scaled_Mag,
    .get_arm_state = SrvDataHub_Get_Arm,
    .get_failsafe = SrvDataHub_Get_Failsafe,
    .get_control_mode = SrvDataHub_Get_ControlMode,
    .get_rc = SrvDataHub_Get_RcChannel,
};

static void SrvDataHub_Init(void)
{
    if (SrvDataHub_Monitor.init_state)
        return;

    memset(&SrvDataHub_Monitor, 0, sizeof(SrvDataHub_Monitor));

    /* init pipe object */
    memset(DataPipe_DataObjAddr(PtlPriIMU_Data), NULL, DataPipe_DataSize(PtlPriIMU_Data));
    IMU_Ptl_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(PtlPriIMU_Data);
    IMU_Ptl_DataPipe.data_size = DataPipe_DataSize(PtlPriIMU_Data);
    IMU_Ptl_DataPipe.trans_finish_cb = SrvComProto_PipeRcTelemtryDataFinish_Callback;
    // DataPipe_Set_RxInterval(&IMU_Ptl_DataPipe, Runtime_MsToUs(5)); /* limit pipe frequence to 200Hz */
    DataPipe_Enable(&IMU_Ptl_DataPipe);

    /* init pipe object */
    memset(DataPipe_DataObjAddr(Proto_Rc), 0, DataPipe_DataSize(Proto_Rc));
    Receiver_ptl_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Proto_Rc);
    Receiver_ptl_DataPipe.data_size = DataPipe_DataSize(Proto_Rc);
    Receiver_ptl_DataPipe.trans_finish_cb = SrvComProto_PipeRcTelemtryDataFinish_Callback;
    // DataPipe_Set_RxInterval(&Receiver_ptl_DataPipe, Runtime_MsToUs(20)); /* limit pipe frequence to 50Hz */
    DataPipe_Enable(&Receiver_ptl_DataPipe);

    memset(&SrvDataHub_Monitor, 0, sizeof(SrvDataHub_Monitor));
    SrvDataHub_Monitor.init_state = true;
}

static void SrvComProto_PipeRcTelemtryDataFinish_Callback(DataPipeObj_TypeDef *obj)
{
    if ((!SrvDataHub_Monitor.init_state) || (obj == NULL))
        return;

    if (obj == &Receiver_ptl_DataPipe)
    {
        SrvDataHub_Monitor.update_reg.bit.rc = true;

        if(SrvDataHub_Monitor.inuse_reg.bit.rc)
            SrvDataHub_Monitor.inuse_reg.bit.rc = false;

        SrvDataHub_Monitor.data.rc_update_time = DataPipe_DataObj(Proto_Rc).time_stamp;
        SrvDataHub_Monitor.data.arm = DataPipe_DataObj(Proto_Rc).arm_state;
        SrvDataHub_Monitor.data.mode = DataPipe_DataObj(Proto_Rc).control_mode;
        SrvDataHub_Monitor.data.buzz = DataPipe_DataObj(Proto_Rc).buzz_state;
        SrvDataHub_Monitor.data.failsafe = DataPipe_DataObj(Proto_Rc).failsafe;
        SrvDataHub_Monitor.data.channel_sum = DataPipe_DataObj(Proto_Rc).channel_sum;

        for(uint8_t i = 0; i < SrvDataHub_Monitor.data.channel_sum; i++)
        {
            SrvDataHub_Monitor.data.ch[i] = DataPipe_DataObj(Proto_Rc).channel[i];
        }

        for()
        {
            SrvDataHub_Monitor.data.ch
        }

        SrvDataHub_Monitor.update_reg.bit.rc = false;
    }
    else if (obj == &IMU_Ptl_DataPipe)
    {
        SrvDataHub_Monitor.update_reg.bit.raw_imu = true;
        SrvDataHub_Monitor.update_reg.bit.scaled_imu = true;

        if (SrvDataHub_Monitor.inuse_reg.bit.raw_imu)
            SrvDataHub_Monitor.inuse_reg.bit.raw_imu = false;

        if (SrvDataHub_Monitor.update_reg.bit.scaled_imu)
            SrvDataHub_Monitor.update_reg.bit.scaled_imu = false;

        SrvDataHub_Monitor.data.imu_update_time = DataPipe_DataObj(PtlPriIMU_Data).data.time_stamp;
        SrvDataHub_Monitor.data.acc_scale = DataPipe_DataObj(PtlPriIMU_Data).data.acc_scale;
        SrvDataHub_Monitor.data.gyr_scale = DataPipe_DataObj(PtlPriIMU_Data).data.gyr_scale;
        SrvDataHub_Monitor.data.imu_temp = DataPipe_DataObj(PtlPriIMU_Data).data.tempera;

        SrvDataHub_Monitor.data.flt_acc_x = DataPipe_DataObj(PtlPriIMU_Data).data.flt_acc[Axis_X];
        SrvDataHub_Monitor.data.flt_acc_y = DataPipe_DataObj(PtlPriIMU_Data).data.flt_acc[Axis_Y];
        SrvDataHub_Monitor.data.flt_acc_z = DataPipe_DataObj(PtlPriIMU_Data).data.flt_acc[Axis_Z];

        SrvDataHub_Monitor.data.flt_gyr_x = DataPipe_DataObj(PtlPriIMU_Data).data.flt_gyr[Axis_X];
        SrvDataHub_Monitor.data.flt_gyr_y = DataPipe_DataObj(PtlPriIMU_Data).data.flt_gyr[Axis_Y];
        SrvDataHub_Monitor.data.flt_gyr_z = DataPipe_DataObj(PtlPriIMU_Data).data.flt_gyr[Axis_Z];

        SrvDataHub_Monitor.data.org_acc_x = DataPipe_DataObj(PtlPriIMU_Data).data.org_acc[Axis_X];
        SrvDataHub_Monitor.data.org_acc_y = DataPipe_DataObj(PtlPriIMU_Data).data.org_acc[Axis_Y];
        SrvDataHub_Monitor.data.org_acc_z = DataPipe_DataObj(PtlPriIMU_Data).data.org_acc[Axis_Z];

        SrvDataHub_Monitor.data.org_gyr_x = DataPipe_DataObj(PtlPriIMU_Data).data.org_gyr[Axis_X];
        SrvDataHub_Monitor.data.org_gyr_y = DataPipe_DataObj(PtlPriIMU_Data).data.org_gyr[Axis_Y];
        SrvDataHub_Monitor.data.org_gyr_z = DataPipe_DataObj(PtlPriIMU_Data).data.org_gyr[Axis_Z];

        SrvDataHub_Monitor.update_reg.bit.raw_imu = false;
        SrvDataHub_Monitor.update_reg.bit.scaled_imu = false;
    }
}

static bool SrvDataHub_Get_Raw_IMU(uint32_t *time_stamp, float *acc_scale, float *gyr_scale, float *acc_x, float *acc_y, float *acc_z, float *gyr_x, float *gyr_y, float *gyr_z, float *tmpr)
{
    if ((time_stamp == NULL) ||
        (acc_x == NULL) ||
        (acc_y == NULL) ||
        (acc_z == NULL) ||
        (gyr_x == NULL) ||
        (gyr_y == NULL) ||
        (gyr_z == NULL) ||
        (tmpr == NULL))
        return false;

reupdate_raw_imu:
    SrvDataHub_Monitor.inuse_reg.bit.raw_imu = true;

    *time_stamp = SrvDataHub_Monitor.data.imu_update_time;
    *acc_scale = SrvDataHub_Monitor.data.acc_scale;
    *gyr_scale = SrvDataHub_Monitor.data.gyr_scale;
    *acc_x = SrvDataHub_Monitor.data.org_acc_x;
    *acc_y = SrvDataHub_Monitor.data.org_acc_y;
    *acc_z = SrvDataHub_Monitor.data.org_acc_z;
    *gyr_x = SrvDataHub_Monitor.data.org_gyr_x;
    *gyr_y = SrvDataHub_Monitor.data.org_gyr_y;
    *gyr_z = SrvDataHub_Monitor.data.org_gyr_z;
    *tmpr = SrvDataHub_Monitor.data.imu_temp;

    if (!SrvDataHub_Monitor.inuse_reg.bit.raw_imu)
        goto reupdate_raw_imu;

    SrvDataHub_Monitor.inuse_reg.bit.raw_imu = false;

    return true;
}

static bool SrvDataHub_Get_Scaled_IMU(uint32_t *time_stamp, float *acc_scale, float *gyr_scale, float *acc_x, float *acc_y, float *acc_z, float *gyr_x, float *gyr_y, float *gyr_z, float *tmpr)
{
    if ((time_stamp == NULL) ||
        (acc_x == NULL) ||
        (acc_y == NULL) ||
        (acc_z == NULL) ||
        (gyr_x == NULL) ||
        (gyr_y == NULL) ||
        (gyr_z == NULL) ||
        (tmpr == NULL))
        return false;

reupdate_scaled_imu:
    SrvDataHub_Monitor.inuse_reg.bit.scaled_imu = true;

    *time_stamp = SrvDataHub_Monitor.data.imu_update_time;
    *acc_scale = SrvDataHub_Monitor.data.acc_scale;
    *gyr_scale = SrvDataHub_Monitor.data.gyr_scale;
    *acc_x = SrvDataHub_Monitor.data.flt_acc_x;
    *acc_y = SrvDataHub_Monitor.data.flt_acc_y;
    *acc_z = SrvDataHub_Monitor.data.flt_acc_z;
    *gyr_x = SrvDataHub_Monitor.data.flt_gyr_x;
    *gyr_y = SrvDataHub_Monitor.data.flt_gyr_y;
    *gyr_z = SrvDataHub_Monitor.data.flt_gyr_z;
    *tmpr = SrvDataHub_Monitor.data.imu_temp;

    if (!SrvDataHub_Monitor.inuse_reg.bit.scaled_imu)
        goto reupdate_scaled_imu;

    SrvDataHub_Monitor.inuse_reg.bit.scaled_imu = false;

    return true;
}

static bool SrvDataHub_Get_Raw_Mag(uint32_t *time_stamp, float *scale, float *mag_x, float *mag_y, float *mag_z)
{
    if ((time_stamp == NULL) ||
        (mag_x == NULL) ||
        (mag_y == NULL) ||
        (mag_z == NULL))
        return false;

    return true;
}

static bool SrvDataHub_Get_Scaled_Mag(uint32_t *time_stamp, float *scale, float *mag_x, float *mag_y, float *mag_z)
{
    if ((time_stamp == NULL) ||
        (mag_x == NULL) ||
        (mag_y == NULL) ||
        (mag_z == NULL))
        return false;

    return true;
}

static bool SrvDataHub_Get_Arm(bool *arm)
{
    if(arm == NULL)
        return false;

reupdate_arm:
    SrvDataHub_Monitor.inuse_reg.bit.rc = true;
    *arm = SrvDataHub_Monitor.data.arm;

    if(!SrvDataHub_Monitor.inuse_reg.bit.rc)
        goto reupdate_arm;

    SrvDataHub_Monitor.inuse_reg.bit.rc = false;

    return true;
}

static bool SrvDataHub_Get_Failsafe(bool *failsafe)
{
    if(failsafe == NULL)
        return false;

reupdate_failsafe:
    SrvDataHub_Monitor.inuse_reg.bit.rc = true;
    *failsafe = SrvDataHub_Monitor.data.failsafe;

    if(!SrvDataHub_Monitor.inuse_reg.bit.rc)
        goto reupdate_failsafe;

    SrvDataHub_Monitor.inuse_reg.bit.rc = false;

    return true;
}

static bool SrvDataHub_Get_ControlMode(uint8_t *mode)
{
    if(mode == NULL)
        return false;

reupdate_control_mode:
    SrvDataHub_Monitor.inuse_reg.bit.rc = true;
    *mode = SrvDataHub_Monitor.data.mode;

    if(!SrvDataHub_Monitor.inuse_reg.bit.rc)
        goto reupdate_control_mode;

    SrvDataHub_Monitor.inuse_reg.bit.rc = false;

    return true;
}

static bool SrvDataHub_Get_RcChannel(uint32_t *time_stamp, uint16_t *ch, uint8_t *ch_cum)
{
    if((time_stamp == NULL) || (ch == NULL) || (ch_sum == NULL))
        return false;

reupdate_rc_channel:
    SrvDataHub_Monitor.inuse_reg.bit.rc = true;
    *time_stamp = SrvDataHub_Monitor.data.rc_update_time;
    *ch_sum = SrvDataHub_Monitor.data.channel_sum;

    for(uint8_t i = 0; i < SrvDataHub_Monitor.data.channel_sum; i++)
    {
        ch[i] = SrvDataHub_Monitor.data.ch[i];
    }

    if(!SrvDataHub_Monitor.inuse_reg.bit.rc)
        goto reupdate_rc_channel;

    SrvDataHub_Monitor.inuse_reg.bit.rc  = false;

    return true;
}
