#include "Srv_DataHub.h"

/* internal variable */
SrvDataHub_Monitor_TypeDef SrvDataHub_Monitor = {
    .init_state = false,
};

/* Pipe Object */
DataPipe_CreateDataObj(SrvIMU_UnionData_TypeDef, PtlIMU_Data);
DataPipe_CreateDataObj(SrvActuatorPipeData_TypeDef, PtlActuator_Data);
DataPipe_CreateDataObj(SrvRecever_RCSig_TypeDef, Proto_Rc);
DataPipe_CreateDataObj(SrvSensorMonitor_GenReg_TypeDef, Sensor_Enable);
DataPipe_CreateDataObj(SrvSensorMonitor_GenReg_TypeDef, Sensor_Init);
DataPipe_CreateDataObj(IMUAtt_TypeDef, Hub_Attitude);

/* internal function */
static void SrvDataHub_PipeRcTelemtryDataFinish_Callback(DataPipeObj_TypeDef *obj);
static void SrvDataHub_SensorState_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj);
static void SrvDataHub_IMU_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj);
static void SrvDataHub_Actuator_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj);
static void SrvDataHub_Attitude_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj);

/* external function */
static void SrvDataHub_Init(void);
static bool SrvDataHub_Get_Raw_IMU(uint32_t *time_stamp, float *acc_scale, float *gyr_scale, float *acc_x, float *acc_y, float *acc_z, float *gyr_x, float *gyr_y, float *gyr_z, float *tmpr, uint8_t *err);
static bool SrvDataHub_Get_Scaled_IMU(uint32_t *time_stamp, float *acc_scale, float *gyr_scale, float *acc_x, float *acc_y, float *acc_z, float *gyr_x, float *gyr_y, float *gyr_z, float *tmpr, uint8_t *err);
static bool SrvDataHub_Get_Raw_Mag(uint32_t *time_stamp, float *scale, float *mag_x, float *mag_y, float *mag_z, uint8_t *err);
static bool SrvDataHub_Get_Scaled_Mag(uint32_t *time_stamp, float *scale, float *mag_x, float *mag_y, float *mag_z, uint8_t *err);
static bool SrvDataHub_Get_Arm(bool *arm);
static bool SrvDataHub_Get_Failsafe(bool *failsafe);
static bool SrvDataHub_Get_ControlMode(uint8_t *mode);
static bool SrvDataHub_Get_RcChannel(uint32_t *time_stamp, uint16_t *ch, uint8_t *ch_cum);
static bool SrvDataHub_Get_GimbalPercent(uint16_t *gimbal);
static bool SrvDataHub_Get_MotoChannel(uint32_t *time_stamp, uint8_t *cnt, uint16_t *moto_ch, uint8_t *moto_dir);
static bool SrvDataHub_Get_ServoChannel(uint32_t *time_stamp, uint8_t *cnt, uint16_t *servo_ch, uint8_t *servo_dir);
static bool SrvDataHub_Get_IMU_InitState(bool *state);
static bool SrvDataHub_Get_Mag_InitState(bool *state);
static bool SrvDataHub_Get_Attitude(uint32_t *time_stamp, float *pitch, float *roll, float *yaw, float *q0, float *q1, float *q2, float *q3);
static bool SrvDataHub_Get_TunningState(uint32_t *time_stamp, bool *state, uint32_t *port_addr);
static bool SrvDataHub_Get_ConfigratorAttachState(bool *state);

static bool SrvDataHub_Set_ConfigratorAttachState(uint32_t time_stamp, bool state);
static bool SrvDataHub_Set_TunningState(uint32_t time_stamp, bool state, uint32_t port_addr);


/* external variable */
SrvDataHub_TypeDef SrvDataHub = {
    .init = SrvDataHub_Init,
    .get_raw_imu = SrvDataHub_Get_Raw_IMU,
    .get_scaled_imu = SrvDataHub_Get_Scaled_IMU,
    .get_attitude = SrvDataHub_Get_Attitude,
    .get_raw_mag = SrvDataHub_Get_Raw_Mag,
    .get_scaled_mag = SrvDataHub_Get_Scaled_Mag,
    .get_arm_state = SrvDataHub_Get_Arm,
    .get_failsafe = SrvDataHub_Get_Failsafe,
    .get_control_mode = SrvDataHub_Get_ControlMode,
    .get_rc = SrvDataHub_Get_RcChannel,
    .get_gimbal_percent = SrvDataHub_Get_GimbalPercent,
    .get_moto = SrvDataHub_Get_MotoChannel,
    .get_servo = SrvDataHub_Get_ServoChannel,
    .get_imu_init_state = SrvDataHub_Get_IMU_InitState,
    .get_mag_init_state = SrvDataHub_Get_Mag_InitState,
    .get_tunning_state =  SrvDataHub_Get_TunningState,
    .get_configrator_attach_state =  SrvDataHub_Get_ConfigratorAttachState,

    .set_tunning_state = SrvDataHub_Set_TunningState,
    .set_configrator_state = SrvDataHub_Set_ConfigratorAttachState,
};

static void SrvDataHub_Init(void)
{
    if (SrvDataHub_Monitor.init_state)
        return;

    memset(&SrvDataHub_Monitor, 0, sizeof(SrvDataHub_Monitor));

    /* init pipe object */
    memset(DataPipe_DataObjAddr(PtlIMU_Data), NULL, DataPipe_DataSize(PtlIMU_Data));
    IMU_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(PtlIMU_Data);
    IMU_hub_DataPipe.data_size = DataPipe_DataSize(PtlIMU_Data);
    IMU_hub_DataPipe.trans_finish_cb = SrvDataHub_IMU_DataPipe_Finish_Callback;
    DataPipe_Enable(&IMU_hub_DataPipe);

    /* init pipe object */
    memset(DataPipe_DataObjAddr(Proto_Rc), 0, DataPipe_DataSize(Proto_Rc));
    Receiver_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Proto_Rc);
    Receiver_hub_DataPipe.data_size = DataPipe_DataSize(Proto_Rc);
    Receiver_hub_DataPipe.trans_finish_cb = SrvDataHub_PipeRcTelemtryDataFinish_Callback;
    DataPipe_Enable(&Receiver_hub_DataPipe);

    memset(DataPipe_DataObjAddr(Sensor_Enable), 0, DataPipe_DataSize(Sensor_Enable));
    SensorEnableState_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Sensor_Enable);
    SensorEnableState_hub_DataPipe.data_size = DataPipe_DataSize(Sensor_Enable);
    SensorEnableState_hub_DataPipe.trans_finish_cb = SrvDataHub_SensorState_DataPipe_Finish_Callback;
    DataPipe_Enable(&SensorEnableState_hub_DataPipe);

    memset(DataPipe_DataObjAddr(Sensor_Init), 0, DataPipe_DataSize(Sensor_Init));
    SensorInitState_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Sensor_Init);
    SensorInitState_hub_DataPipe.data_size = DataPipe_DataSize(Sensor_Init);
    SensorInitState_hub_DataPipe.trans_finish_cb = SrvDataHub_SensorState_DataPipe_Finish_Callback;
    DataPipe_Enable(&SensorInitState_hub_DataPipe);

    memset(DataPipe_DataObjAddr(PtlActuator_Data), 0, DataPipe_DataSize(PtlActuator_Data));
    Actuator_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(PtlActuator_Data);
    Actuator_hub_DataPipe.data_size = DataPipe_DataSize(PtlActuator_Data);
    Actuator_hub_DataPipe.trans_finish_cb = SrvDataHub_Actuator_DataPipe_Finish_Callback;
    DataPipe_Enable(&Actuator_hub_DataPipe);
    
    memset(DataPipe_DataObjAddr(Hub_Attitude), 0, DataPipe_DataSize(Hub_Attitude));
    Attitude_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Hub_Attitude);
    Attitude_hub_DataPipe.data_size = DataPipe_DataSize(Hub_Attitude);
    Attitude_hub_DataPipe.trans_finish_cb = SrvDataHub_Attitude_DataPipe_Finish_Callback;
    DataPipe_Enable(&Attitude_hub_DataPipe);

    memset(&SrvDataHub_Monitor, 0, sizeof(SrvDataHub_Monitor));
    SrvDataHub_Monitor.init_state = true;
}

static void SrvDataHub_Attitude_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj)
{
    if(obj == &Attitude_hub_DataPipe)
    {
        SrvDataHub_Monitor.update_reg.bit.attitude = true;

        if(SrvDataHub_Monitor.inuse_reg.bit.attitude)
            SrvDataHub_Monitor.inuse_reg.bit.attitude = false;

        SrvDataHub_Monitor.data.att_update_time = DataPipe_DataObj(Hub_Attitude).time_stamp;
        SrvDataHub_Monitor.data.att_pitch = DataPipe_DataObj(Hub_Attitude).pitch;
        SrvDataHub_Monitor.data.att_roll = DataPipe_DataObj(Hub_Attitude).roll;
        SrvDataHub_Monitor.data.att_yaw = DataPipe_DataObj(Hub_Attitude).yaw;
        SrvDataHub_Monitor.data.att_q0 = DataPipe_DataObj(Hub_Attitude).q0;
        SrvDataHub_Monitor.data.att_q1 = DataPipe_DataObj(Hub_Attitude).q1;
        SrvDataHub_Monitor.data.att_q2 = DataPipe_DataObj(Hub_Attitude).q2;
        SrvDataHub_Monitor.data.att_q3 = DataPipe_DataObj(Hub_Attitude).q3;
        SrvDataHub_Monitor.data.att_error_code = DataPipe_DataObj(Hub_Attitude).err_code;
    
        SrvDataHub_Monitor.update_reg.bit.attitude = false;
    }
}

static void SrvDataHub_Actuator_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj)
{
    if (obj == &Actuator_hub_DataPipe)
    {
        SrvDataHub_Monitor.update_reg.bit.actuator = true;

        if (SrvDataHub_Monitor.inuse_reg.bit.actuator)
            SrvDataHub_Monitor.inuse_reg.bit.actuator = false;

        SrvDataHub_Monitor.data.actuator_update_time = DataPipe_DataObj(PtlActuator_Data).time_stamp;
        SrvDataHub_Monitor.data.moto_num = DataPipe_DataObj(PtlActuator_Data).moto_cnt;
        SrvDataHub_Monitor.data.servo_num = DataPipe_DataObj(PtlActuator_Data).servo_cnt;

        memset(SrvDataHub_Monitor.data.moto_dir, 0, sizeof(SrvDataHub_Monitor.data.moto_dir));
        memset(SrvDataHub_Monitor.data.servo_dir, 0, sizeof(SrvDataHub_Monitor.data.servo_dir));
        memset(SrvDataHub_Monitor.data.moto, 0, sizeof(SrvDataHub_Monitor.data.moto));
        memset(SrvDataHub_Monitor.data.servo, 0, sizeof(SrvDataHub_Monitor.data.servo));

        for (uint8_t moto_i = 0; moto_i < SrvDataHub_Monitor.data.moto_num; moto_i++)
        {
            SrvDataHub_Monitor.data.moto[moto_i] = DataPipe_DataObj(PtlActuator_Data).moto[moto_i];
        }

        for (uint8_t servo_i = 0; servo_i < SrvDataHub_Monitor.data.servo_num; servo_i++)
        {
            SrvDataHub_Monitor.data.servo[servo_i] = DataPipe_DataObj(PtlActuator_Data).servo[servo_i];
        }

        SrvDataHub_Monitor.update_reg.bit.actuator = false;
    }
}

static void SrvDataHub_IMU_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj)
{
    if (obj == &IMU_hub_DataPipe)
    {
        SrvDataHub_Monitor.update_reg.bit.raw_imu = true;
        SrvDataHub_Monitor.update_reg.bit.scaled_imu = true;

        if (SrvDataHub_Monitor.inuse_reg.bit.raw_imu)
            SrvDataHub_Monitor.inuse_reg.bit.raw_imu = false;

        if (SrvDataHub_Monitor.update_reg.bit.scaled_imu)
            SrvDataHub_Monitor.update_reg.bit.scaled_imu = false;

        SrvDataHub_Monitor.data.imu_update_time = DataPipe_DataObj(PtlIMU_Data).data.time_stamp;
        SrvDataHub_Monitor.data.acc_scale = DataPipe_DataObj(PtlIMU_Data).data.acc_scale;
        SrvDataHub_Monitor.data.gyr_scale = DataPipe_DataObj(PtlIMU_Data).data.gyr_scale;
        SrvDataHub_Monitor.data.imu_temp = DataPipe_DataObj(PtlIMU_Data).data.tempera;

        SrvDataHub_Monitor.data.flt_acc_x = DataPipe_DataObj(PtlIMU_Data).data.flt_acc[Axis_X];
        SrvDataHub_Monitor.data.flt_acc_y = DataPipe_DataObj(PtlIMU_Data).data.flt_acc[Axis_Y];
        SrvDataHub_Monitor.data.flt_acc_z = DataPipe_DataObj(PtlIMU_Data).data.flt_acc[Axis_Z];

        SrvDataHub_Monitor.data.flt_gyr_x = DataPipe_DataObj(PtlIMU_Data).data.flt_gyr[Axis_X];
        SrvDataHub_Monitor.data.flt_gyr_y = DataPipe_DataObj(PtlIMU_Data).data.flt_gyr[Axis_Y];
        SrvDataHub_Monitor.data.flt_gyr_z = DataPipe_DataObj(PtlIMU_Data).data.flt_gyr[Axis_Z];

        SrvDataHub_Monitor.data.org_acc_x = DataPipe_DataObj(PtlIMU_Data).data.org_acc[Axis_X];
        SrvDataHub_Monitor.data.org_acc_y = DataPipe_DataObj(PtlIMU_Data).data.org_acc[Axis_Y];
        SrvDataHub_Monitor.data.org_acc_z = DataPipe_DataObj(PtlIMU_Data).data.org_acc[Axis_Z];

        SrvDataHub_Monitor.data.org_gyr_x = DataPipe_DataObj(PtlIMU_Data).data.org_gyr[Axis_X];
        SrvDataHub_Monitor.data.org_gyr_y = DataPipe_DataObj(PtlIMU_Data).data.org_gyr[Axis_Y];
        SrvDataHub_Monitor.data.org_gyr_z = DataPipe_DataObj(PtlIMU_Data).data.org_gyr[Axis_Z];

        SrvDataHub_Monitor.data.imu_error_code = DataPipe_DataObj(PtlIMU_Data).data.error_code;

        SrvDataHub_Monitor.update_reg.bit.raw_imu = false;
        SrvDataHub_Monitor.update_reg.bit.scaled_imu = false;
    }
}

static void SrvDataHub_SensorState_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj)
{
    if((obj == &SensorInitState_hub_DataPipe) || (obj == &SensorEnableState_hub_DataPipe))
    {
        SrvDataHub_Monitor.update_reg.bit.imu_init = true;
        SrvDataHub_Monitor.update_reg.bit.mag_init = true;

        if (SrvDataHub_Monitor.inuse_reg.bit.imu_init)
            SrvDataHub_Monitor.inuse_reg.bit.imu_init = false;

        if(SrvDataHub_Monitor.inuse_reg.bit.mag_init)
            SrvDataHub_Monitor.inuse_reg.bit.mag_init = false;

        if(obj == &SensorInitState_hub_DataPipe)
        {
            SrvDataHub_Monitor.data.mag_enabled = DataPipe_DataObj(Sensor_Enable).bit.mag;
            SrvDataHub_Monitor.data.baro_enabled = DataPipe_DataObj(Sensor_Enable).bit.baro;
            SrvDataHub_Monitor.data.tof_enabled = DataPipe_DataObj(Sensor_Enable).bit.tof;
            SrvDataHub_Monitor.data.gnss_enable = DataPipe_DataObj(Sensor_Enable).bit.gnss;
        }
        else
        {
            SrvDataHub_Monitor.data.imu_init_state = DataPipe_DataObj(Sensor_Init).bit.imu;
            
            if(SrvDataHub_Monitor.data.mag_enabled)
            {
                SrvDataHub_Monitor.data.mag_init_state = DataPipe_DataObj(Sensor_Init).bit.mag;
            }
            else
                SrvDataHub_Monitor.data.mag_init_state = false;

            if(SrvDataHub_Monitor.data.baro_enabled)
            {
                SrvDataHub_Monitor.data.baro_init_state = DataPipe_DataObj(Sensor_Init).bit.baro;
            }
            else
                SrvDataHub_Monitor.data.baro_init_state = false;

            if(SrvDataHub_Monitor.data.tof_enabled)
            {
                SrvDataHub_Monitor.data.tof_init_state = DataPipe_DataObj(Sensor_Init).bit.tof;
            }
            else
                SrvDataHub_Monitor.data.tof_init_state = false;
        }

        SrvDataHub_Monitor.update_reg.bit.imu_init = false;
        SrvDataHub_Monitor.update_reg.bit.mag_init = false;
    }
}

static void SrvDataHub_PipeRcTelemtryDataFinish_Callback(DataPipeObj_TypeDef *obj)
{
    if ((!SrvDataHub_Monitor.init_state) || (obj == NULL))
        return;

    if (obj == &Receiver_hub_DataPipe)
    {
        SrvDataHub_Monitor.update_reg.bit.rc = true;

        if (SrvDataHub_Monitor.inuse_reg.bit.rc)
            SrvDataHub_Monitor.inuse_reg.bit.rc = false;

        SrvDataHub_Monitor.data.rc_update_time = DataPipe_DataObj(Proto_Rc).time_stamp;
        SrvDataHub_Monitor.data.arm = DataPipe_DataObj(Proto_Rc).arm_state;
        SrvDataHub_Monitor.data.mode = DataPipe_DataObj(Proto_Rc).control_mode;
        SrvDataHub_Monitor.data.buzz = DataPipe_DataObj(Proto_Rc).buzz_state;
        SrvDataHub_Monitor.data.failsafe = DataPipe_DataObj(Proto_Rc).failsafe;
        SrvDataHub_Monitor.data.channel_sum = DataPipe_DataObj(Proto_Rc).channel_sum;

        for (uint8_t i = 0; i < SrvDataHub_Monitor.data.channel_sum; i++)
        {
            SrvDataHub_Monitor.data.ch[i] = DataPipe_DataObj(Proto_Rc).channel[i];

            if (i < 4)
                SrvDataHub_Monitor.data.gimbal[i] = DataPipe_DataObj(Proto_Rc).gimbal_percent[i];
        }

        SrvDataHub_Monitor.update_reg.bit.rc = false;
    }
 }

static bool SrvDataHub_Get_Raw_IMU(uint32_t *time_stamp, float *acc_scale, float *gyr_scale, float *acc_x, float *acc_y, float *acc_z, float *gyr_x, float *gyr_y, float *gyr_z, float *tmpr, uint8_t *err)
{
    if ((time_stamp == NULL) ||
        (acc_scale == NULL) ||
        (gyr_scale == NULL) ||
        (acc_x == NULL) ||
        (acc_y == NULL) ||
        (acc_z == NULL) ||
        (gyr_x == NULL) ||
        (gyr_y == NULL) ||
        (gyr_z == NULL) ||
        (tmpr == NULL) ||
        (err == NULL))
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
    *err = SrvDataHub_Monitor.data.imu_error_code;

    if (!SrvDataHub_Monitor.inuse_reg.bit.raw_imu)
        goto reupdate_raw_imu;

    SrvDataHub_Monitor.inuse_reg.bit.raw_imu = false;

    return true;
}

static bool SrvDataHub_Get_Scaled_IMU(uint32_t *time_stamp, float *acc_scale, float *gyr_scale, float *acc_x, float *acc_y, float *acc_z, float *gyr_x, float *gyr_y, float *gyr_z, float *tmpr, uint8_t *err)
{
    if ((time_stamp == NULL) ||
        (acc_scale == NULL) ||
        (gyr_scale == NULL) ||
        (acc_x == NULL) ||
        (acc_y == NULL) ||
        (acc_z == NULL) ||
        (gyr_x == NULL) ||
        (gyr_y == NULL) ||
        (gyr_z == NULL) ||
        (tmpr == NULL) ||
        (err == NULL))
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
    *err = SrvDataHub_Monitor.data.imu_error_code;

    if (!SrvDataHub_Monitor.inuse_reg.bit.scaled_imu)
        goto reupdate_scaled_imu;

    SrvDataHub_Monitor.inuse_reg.bit.scaled_imu = false;

    return true;
}

static bool SrvDataHub_Get_Raw_Mag(uint32_t *time_stamp, float *scale, float *mag_x, float *mag_y, float *mag_z, uint8_t *err)
{
    if ((time_stamp == NULL) ||
        (mag_x == NULL) ||
        (mag_y == NULL) ||
        (mag_z == NULL) ||
        (err == NULL))
        return false;

reupdate_raw_mag:
    SrvDataHub_Monitor.inuse_reg.bit.raw_mag = true;

    *time_stamp = SrvDataHub_Monitor.data.mag_update_time;
    *scale = SrvDataHub_Monitor.data.mag_scale;
    *mag_x = SrvDataHub_Monitor.data.org_mag_x;
    *mag_y = SrvDataHub_Monitor.data.org_mag_y;
    *mag_z = SrvDataHub_Monitor.data.org_mag_z;
    *err = SrvDataHub_Monitor.data.mag_error_code;

    if (!SrvDataHub_Monitor.inuse_reg.bit.raw_mag)
        goto reupdate_raw_mag;

    return true;
}

static bool SrvDataHub_Get_Scaled_Mag(uint32_t *time_stamp, float *scale, float *mag_x, float *mag_y, float *mag_z, uint8_t *err)
{
    if ((time_stamp == NULL) ||
        (mag_x == NULL) ||
        (mag_y == NULL) ||
        (mag_z == NULL) ||
        (err == NULL))
        return false;

reupdate_scaled_mag:
    SrvDataHub_Monitor.inuse_reg.bit.scaled_mag = true;

    *time_stamp = SrvDataHub_Monitor.data.mag_update_time;
    *scale = SrvDataHub_Monitor.data.mag_scale;
    *mag_x = SrvDataHub_Monitor.data.flt_mag_x;
    *mag_y = SrvDataHub_Monitor.data.flt_mag_y;
    *mag_z = SrvDataHub_Monitor.data.flt_mag_z;
    *err = SrvDataHub_Monitor.data.mag_error_code;

    if (!SrvDataHub_Monitor.inuse_reg.bit.scaled_mag)
        goto reupdate_scaled_mag;

    return true;
}

static bool SrvDataHub_Get_Attitude(uint32_t *time_stamp, float *pitch, float *roll, float *yaw, float *q0, float *q1, float *q2, float *q3)
{
    if((time_stamp == NULL) || \
       (pitch == NULL) || \
       (roll == NULL) || \
       (yaw == NULL) || \
       (q0 == NULL) || \
       (q1 == NULL) || \
       (q2 == NULL) || \
       (q3 == NULL))
       return false;

reupdate_attitude:
    SrvDataHub_Monitor.inuse_reg.bit.attitude = true; 
    
    *time_stamp = SrvDataHub_Monitor.data.att_update_time;
    *pitch = SrvDataHub_Monitor.data.att_pitch;
    *roll = SrvDataHub_Monitor.data.att_roll;
    *yaw = SrvDataHub_Monitor.data.att_yaw;

    *q0 = SrvDataHub_Monitor.data.att_q0;
    *q1 = SrvDataHub_Monitor.data.att_q1;
    *q2 = SrvDataHub_Monitor.data.att_q2;
    *q3 = SrvDataHub_Monitor.data.att_q3;

    if(!SrvDataHub_Monitor.inuse_reg.bit.attitude)
        goto reupdate_attitude;

    return true;
}

static bool SrvDataHub_Get_Arm(bool *arm)
{
    if (arm == NULL)
        return false;

reupdate_arm:
    SrvDataHub_Monitor.inuse_reg.bit.rc = true;
    *arm = SrvDataHub_Monitor.data.arm;

    if (!SrvDataHub_Monitor.inuse_reg.bit.rc)
        goto reupdate_arm;

    SrvDataHub_Monitor.inuse_reg.bit.rc = false;

    return true;
}

static bool SrvDataHub_Get_Failsafe(bool *failsafe)
{
    if (failsafe == NULL)
        return false;

reupdate_failsafe:
    SrvDataHub_Monitor.inuse_reg.bit.rc = true;
    *failsafe = SrvDataHub_Monitor.data.failsafe;

    if (!SrvDataHub_Monitor.inuse_reg.bit.rc)
        goto reupdate_failsafe;

    SrvDataHub_Monitor.inuse_reg.bit.rc = false;

    return true;
}

static bool SrvDataHub_Get_ControlMode(uint8_t *mode)
{
    if (mode == NULL)
        return false;

reupdate_control_mode:
    SrvDataHub_Monitor.inuse_reg.bit.rc = true;
    *mode = SrvDataHub_Monitor.data.mode;

    if (!SrvDataHub_Monitor.inuse_reg.bit.rc)
        goto reupdate_control_mode;

    SrvDataHub_Monitor.inuse_reg.bit.rc = false;

    return true;
}

static bool SrvDataHub_Get_RcChannel(uint32_t *time_stamp, uint16_t *ch, uint8_t *ch_sum)
{
    if ((time_stamp == NULL) || (ch == NULL) || (ch_sum == NULL))
        return false;

reupdate_rc_channel:
    SrvDataHub_Monitor.inuse_reg.bit.rc = true;
    *time_stamp = SrvDataHub_Monitor.data.rc_update_time;
    *ch_sum = SrvDataHub_Monitor.data.channel_sum;

    for (uint8_t i = 0; i < SrvDataHub_Monitor.data.channel_sum; i++)
    {
        ch[i] = SrvDataHub_Monitor.data.ch[i];
    }

    if (!SrvDataHub_Monitor.inuse_reg.bit.rc)
        goto reupdate_rc_channel;

    SrvDataHub_Monitor.inuse_reg.bit.rc = false;

    return true;
}

static bool SrvDataHub_Get_GimbalPercent(uint16_t *gimbal)
{
    if (gimbal == NULL)
        return false;

reupdate_gimbal:
    SrvDataHub_Monitor.inuse_reg.bit.rc = true;

    for (uint8_t i = 0; i < 4; i++)
    {
        gimbal[i] = SrvDataHub_Monitor.data.gimbal[i];
    }

    if (!SrvDataHub_Monitor.inuse_reg.bit.rc)
        goto reupdate_gimbal;

    SrvDataHub_Monitor.inuse_reg.bit.rc = false;

    return true;
}

static bool SrvDataHub_Get_MotoChannel(uint32_t *time_stamp, uint8_t *cnt, uint16_t *moto_ch, uint8_t *moto_dir)
{
    if ((cnt == NULL) || (moto_ch == NULL))
        return false;

reupdate_moto_channel:
    SrvDataHub_Monitor.inuse_reg.bit.actuator = true;

    *time_stamp = SrvDataHub_Monitor.data.actuator_update_time;
    *cnt = SrvDataHub_Monitor.data.moto_num;

    if (*cnt)
    {
        memcpy(moto_ch, SrvDataHub_Monitor.data.moto, *cnt);
        memcpy(moto_dir, SrvDataHub_Monitor.data.moto_dir, *cnt);
    }

    if (!SrvDataHub_Monitor.inuse_reg.bit.actuator)
        goto reupdate_moto_channel;

    SrvDataHub_Monitor.inuse_reg.bit.actuator = false;

    return true;
}

static bool SrvDataHub_Get_IMU_InitState(bool *state)
{
reupdate_imu_state:
    SrvDataHub_Monitor.inuse_reg.bit.imu_init = true;

    if(state)
    {
        *state = SrvDataHub_Monitor.data.imu_init_state;
        return true;
    }

    if(!SrvDataHub_Monitor.inuse_reg.bit.imu_init)
        goto reupdate_imu_state;

    SrvDataHub_Monitor.inuse_reg.bit.imu_init = false;

    return false;
}

static bool SrvDataHub_Get_Mag_InitState(bool *state)
{
reupdate_mag_state:
    SrvDataHub_Monitor.inuse_reg.bit.mag_init = true;

    if(state)
    {
        *state = SrvDataHub_Monitor.data.mag_init_state;
        return true;
    }

    if(!SrvDataHub_Monitor.inuse_reg.bit.mag_init)
        goto reupdate_mag_state;

    SrvDataHub_Monitor.inuse_reg.bit.mag_init = false;

    return false;
}

static bool SrvDataHub_Get_ServoChannel(uint32_t *time_stamp, uint8_t *cnt, uint16_t *servo_ch, uint8_t *servo_dir)
{
    if ((cnt == NULL) || (servo_ch == NULL) || (servo_dir == NULL))
        return false;

reupdate_servo_channel:
    SrvDataHub_Monitor.inuse_reg.bit.actuator = true;

    *time_stamp = SrvDataHub_Monitor.data.actuator_update_time;
    *cnt = SrvDataHub_Monitor.data.servo_num;

    if (*cnt)
    {
        memcpy(servo_ch, SrvDataHub_Monitor.data.servo, *cnt);
        memcpy(servo_dir, SrvDataHub_Monitor.data.servo_dir, *cnt);
    }

    if (!SrvDataHub_Monitor.inuse_reg.bit.actuator)
        goto reupdate_servo_channel;

    SrvDataHub_Monitor.inuse_reg.bit.actuator = false;

    return true;
}

/* call set tunning under uart/usb/can irq callback */
static bool SrvDataHub_Set_TunningState(uint32_t time_stamp, bool state, uint32_t port_addr)
{
    if((SrvDataHub_Monitor.data.tunning_port_addr == 0) || \
       (port_addr == SrvDataHub_Monitor.data.tunning_port_addr) || \
       !SrvDataHub_Monitor.data.in_tunning)
    {
        if(SrvDataHub_Monitor.inuse_reg.bit.tunning)
            SrvDataHub_Monitor.inuse_reg.bit.tunning = false;

        SrvDataHub_Monitor.update_reg.bit.tunning = true;

        SrvDataHub_Monitor.data.in_tunning = true;
        SrvDataHub_Monitor.data.tunning_heartbeat_timestamp = time_stamp;
        SrvDataHub_Monitor.data.tunning_port_addr = port_addr;

        SrvDataHub_Monitor.update_reg.bit.tunning = false;

        return true;
    }

    return false;
}

static bool SrvDataHub_Get_TunningState(uint32_t *time_stamp, bool *state, uint32_t *port_addr)
{
    if(time_stamp && state && port_addr)
    {
reupdate_tunning_state:
        SrvDataHub_Monitor.inuse_reg.bit.tunning = true;
        
        (*state) = SrvDataHub_Monitor.data.in_tunning;
        (*time_stamp) = SrvDataHub_Monitor.data.tunning_heartbeat_timestamp;
        (*port_addr) = SrvDataHub_Monitor.data.tunning_port_addr;

        if(!SrvDataHub_Monitor.inuse_reg.bit.tunning)
            goto reupdate_tunning_state;

        SrvDataHub_Monitor.inuse_reg.bit.tunning = false;

        return true;
    }

    return false;
}

static bool SrvDataHub_Get_ConfigratorAttachState(bool *state)
{
    if(state)
    {
reupdate_configrator_attach_state:
        SrvDataHub_Monitor.inuse_reg.bit.configrator_attach = true;

        (*state) = SrvDataHub_Monitor.data.attach_configrator;

        if(!SrvDataHub_Monitor.inuse_reg.bit.configrator_attach)
            goto reupdate_configrator_attach_state;

        return true;
    }

    return false;
}

static bool SrvDataHub_Set_ConfigratorAttachState(uint32_t time_stamp, bool state)
{
    if(SrvDataHub_Monitor.inuse_reg.bit.configrator_attach)
       SrvDataHub_Monitor.inuse_reg.bit.configrator_attach = false;

    SrvDataHub_Monitor.update_reg.bit.configrator_attach = true;

    SrvDataHub_Monitor.data.configrator_time_stamp = time_stamp;
    SrvDataHub_Monitor.data.attach_configrator = state;

    SrvDataHub_Monitor.update_reg.bit.configrator_attach = false;

    return true; 
}
