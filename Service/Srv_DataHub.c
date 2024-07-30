#include "Srv_DataHub.h"

#define To_Pipe_TransFinish_Callback(x) ((Pipe_TransFinish_Callback)x)

/* internal variable */
SrvDataHub_Monitor_TypeDef SrvDataHub_Monitor = {
    .init_state = false,
};

/* Pipe Object */
DataPipe_CreateDataObj(SrvIMU_UnionData_TypeDef, PtlIMU_Data);
DataPipe_CreateDataObj(SrvActuatorPipeData_TypeDef, PtlActuator_Data);
DataPipe_CreateDataObj(ControlData_TypeDef, Hub_Telemetry_Rc);
DataPipe_CreateDataObj(SrvSensorMonitor_GenReg_TypeDef, Sensor_Enable);
DataPipe_CreateDataObj(SrvSensorMonitor_GenReg_TypeDef, Sensor_Init);
DataPipe_CreateDataObj(IMUAtt_TypeDef, Hub_Attitude);
DataPipe_CreateDataObj(SrvBaro_UnionData_TypeDef, Hub_Baro_Data);
DataPipe_CreateDataObj(PosData_TypeDef, Hub_Pos);
DataPipe_CreateDataObj(AltData_TypeDef, Hub_Alt);
DataPipe_CreateDataObj(SrvIMU_Range_TypeDef, Hub_PriIMU_Range);
DataPipe_CreateDataObj(SrvIMU_Range_TypeDef, Hub_SecIMU_Range);
DataPipe_CreateDataObj(ExpControlData_TypeDef, Hub_Cnv_CtlData);
DataPipe_CreateDataObj(bool, Hub_VCP_Attach_State);

/* internal function */
static void SrvDataHub_PipeRcTelemtryDataFinish_Callback(DataPipeObj_TypeDef *obj);
static void SrvDataHub_SensorState_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj);
static void SrvDataHub_IMU_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj);
static void SrvDataHub_Actuator_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj);
static void SrvDataHub_Attitude_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj);
static void SrvDataHub_Baro_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj);
static void SrvDataHub_Pos_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj);
static void SrvDatHub_Alt_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj);
static void SrvDataHub_IMU_Range_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj);
static void SrvDataHub_VCPAttach_dataPipe_Finish_Callback(DataPipeObj_TypeDef *obj);
static void SrvDataHub_PipeConvertControlDataFinish_Callback(DataPipeObj_TypeDef *obj);

/* external function */
static void SrvDataHub_Init(void);
static bool SrvDataHub_Get_Raw_IMU(uint32_t *time_stamp, float *acc_scale, float *gyr_scale, float *acc_x, float *acc_y, float *acc_z, float *gyr_x, float *gyr_y, float *gyr_z, float *tmpr, uint8_t *err);
static bool SrvDataHub_Get_Scaled_IMU(uint32_t *time_stamp, float *acc_scale, float *gyr_scale, float *acc_x, float *acc_y, float *acc_z, float *gyr_x, float *gyr_y, float *gyr_z, float *tmpr, uint8_t *err);
static bool SrvDataHub_Get_Raw_Mag(uint32_t *time_stamp, float *scale, float *mag_x, float *mag_y, float *mag_z, uint8_t *err);
static bool SrvDataHub_Get_Scaled_Mag(uint32_t *time_stamp, float *scale, float *mag_x, float *mag_y, float *mag_z, uint8_t *err);
static bool SrvDataHub_Get_Arm(bool *arm);
static bool SrvDataHub_Get_Failsafe(bool *failsafe);
static bool SrvDataHub_Get_Telemetry_ControlData(ControlData_TypeDef *data);
static bool SrvDataHub_Get_MotoChannel(uint32_t *time_stamp, uint8_t *cnt, uint16_t *moto_ch, uint8_t *moto_dir);
static bool SrvDataHub_Get_ServoChannel(uint32_t *time_stamp, uint8_t *cnt, uint16_t *servo_ch, uint8_t *servo_dir);
static bool SrvDataHub_Get_IMU_InitState(bool *state);
static bool SrvDataHub_Get_Mag_InitState(bool *state);
static bool SrvDataHub_Get_Bar_InitState(bool *state);
static bool SrvDataHub_Get_Scaled_Baro(uint32_t *time_stamp, float *baro_pressure, float *baro_alt, float *baro_alt_offset, float *tempra, uint8_t *error);
static bool SrvDataHub_Get_Attitude(uint32_t *time_stamp, float *pitch, float *roll, float *yaw, float *q0, float *q1, float *q2, float *q3, bool *flip_over);
static bool SrvDataHub_Get_VCPAttach_State(bool *state);
static bool SrvDataHub_Get_CLI_State(bool *state);
static bool SrvDataHub_Get_Upgrade_State(bool *state);
static bool SrvDataHub_Get_PriIMU_Range(uint8_t *acc_range, uint16_t *gyr_range);
static bool SrvDataHub_Get_SecIMU_Range(uint8_t *acc_range, uint16_t *gyr_range);
static bool SrvDataHub_Get_RelativeAlt(uint32_t *time_stamp, float *alt);
static bool SrvDataHub_Get_Convert_ControlData(uint32_t *time_stamp, bool *arm, bool *failsafe, uint8_t *mode, float *pitch, float *roll, float *gx, float *gy, float *gz);

static bool SrvDataHub_Set_CLI_State(bool state);
static bool SrvDataHub_Set_Upgrade_State(bool state);

/* external variable */
SrvDataHub_TypeDef SrvDataHub = {
    .init = SrvDataHub_Init,
    .get_raw_imu = SrvDataHub_Get_Raw_IMU,
    .get_scaled_imu = SrvDataHub_Get_Scaled_IMU,
    .get_pri_imu_range = SrvDataHub_Get_PriIMU_Range,
    .get_sec_imu_range = SrvDataHub_Get_SecIMU_Range,
    .get_attitude = SrvDataHub_Get_Attitude,
    .get_raw_mag = SrvDataHub_Get_Raw_Mag,
    .get_scaled_mag = SrvDataHub_Get_Scaled_Mag,
    .get_baro_altitude = SrvDataHub_Get_Scaled_Baro,
    .get_arm_state = SrvDataHub_Get_Arm,
    .get_failsafe = SrvDataHub_Get_Failsafe,
    .get_rc_control_data = SrvDataHub_Get_Telemetry_ControlData,
    .get_moto = SrvDataHub_Get_MotoChannel,
    .get_servo = SrvDataHub_Get_ServoChannel,
    .get_imu_init_state = SrvDataHub_Get_IMU_InitState,
    .get_mag_init_state = SrvDataHub_Get_Mag_InitState,
    .get_baro_init_state = SrvDataHub_Get_Bar_InitState,
    .get_vcp_attach_state = SrvDataHub_Get_VCPAttach_State,
    .get_cli_state = SrvDataHub_Get_CLI_State,
    .get_upgrade_state = SrvDataHub_Get_Upgrade_State,
    .get_relative_alt = SrvDataHub_Get_RelativeAlt,
    .get_cnv_control_data = SrvDataHub_Get_Convert_ControlData,

    .set_cli_state = SrvDataHub_Set_CLI_State,
    .set_upgrade_state = SrvDataHub_Set_Upgrade_State,
};

static void SrvDataHub_Init(void)
{
    if (SrvDataHub_Monitor.init_state)
        return;

    memset(&SrvDataHub_Monitor, 0, sizeof(SrvDataHub_Monitor));

    /* init pipe object */
    memset(DataPipe_DataObjAddr(PtlIMU_Data), 0, DataPipe_DataSize(PtlIMU_Data));
    IMU_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(PtlIMU_Data);
    IMU_hub_DataPipe.data_size = DataPipe_DataSize(PtlIMU_Data);
    IMU_hub_DataPipe.trans_finish_cb = To_Pipe_TransFinish_Callback(SrvDataHub_IMU_DataPipe_Finish_Callback);
    DataPipe_Enable(&IMU_hub_DataPipe);

    memset(DataPipe_DataObjAddr(Hub_PriIMU_Range), 0, DataPipe_DataSize(Hub_PriIMU_Range));
    IMU_PriRange_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Hub_PriIMU_Range);
    IMU_PriRange_hub_DataPipe.data_size = DataPipe_DataSize(Hub_PriIMU_Range);
    IMU_PriRange_hub_DataPipe.trans_finish_cb = To_Pipe_TransFinish_Callback(SrvDataHub_IMU_Range_DataPipe_Finish_Callback);
    DataPipe_Enable(&IMU_PriRange_hub_DataPipe);

#if (IMU_CNT == 2)
    memset(DataPipe_DataObjAddr(Hub_SecIMU_Range), 0, DataPipe_DataSize(Hub_SecIMU_Range));
    IMU_SecRange_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Hub_SecIMU_Range);
    IMU_SecRange_hub_DataPipe.data_size = DataPipe_DataSize(Hub_SecIMU_Range);
    IMU_SecRange_hub_DataPipe.trans_finish_cb = To_Pipe_TransFinish_Callback(SrvDataHub_IMU_Range_DataPipe_Finish_Callback);
    DataPipe_Enable(&IMU_SecRange_hub_DataPipe);
#endif

    /* init pipe object */
    memset(DataPipe_DataObjAddr(Hub_Telemetry_Rc), 0, DataPipe_DataSize(Hub_Telemetry_Rc));
    Receiver_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Hub_Telemetry_Rc);
    Receiver_hub_DataPipe.data_size = DataPipe_DataSize(Hub_Telemetry_Rc);
    Receiver_hub_DataPipe.trans_finish_cb = To_Pipe_TransFinish_Callback(SrvDataHub_PipeRcTelemtryDataFinish_Callback);
    DataPipe_Enable(&Receiver_hub_DataPipe);

    memset(DataPipe_DataObjAddr(Sensor_Enable), 0, DataPipe_DataSize(Sensor_Enable));
    SensorEnableState_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Sensor_Enable);
    SensorEnableState_hub_DataPipe.data_size = DataPipe_DataSize(Sensor_Enable);
    SensorEnableState_hub_DataPipe.trans_finish_cb = To_Pipe_TransFinish_Callback(SrvDataHub_SensorState_DataPipe_Finish_Callback);
    DataPipe_Enable(&SensorEnableState_hub_DataPipe);

    memset(DataPipe_DataObjAddr(Sensor_Init), 0, DataPipe_DataSize(Sensor_Init));
    SensorInitState_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Sensor_Init);
    SensorInitState_hub_DataPipe.data_size = DataPipe_DataSize(Sensor_Init);
    SensorInitState_hub_DataPipe.trans_finish_cb = To_Pipe_TransFinish_Callback(SrvDataHub_SensorState_DataPipe_Finish_Callback);
    DataPipe_Enable(&SensorInitState_hub_DataPipe);

    memset(DataPipe_DataObjAddr(PtlActuator_Data), 0, DataPipe_DataSize(PtlActuator_Data));
    Actuator_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(PtlActuator_Data);
    Actuator_hub_DataPipe.data_size = DataPipe_DataSize(PtlActuator_Data);
    Actuator_hub_DataPipe.trans_finish_cb = To_Pipe_TransFinish_Callback(SrvDataHub_Actuator_DataPipe_Finish_Callback);
    DataPipe_Enable(&Actuator_hub_DataPipe);
    
    memset(DataPipe_DataObjAddr(Hub_Attitude), 0, DataPipe_DataSize(Hub_Attitude));
    Attitude_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Hub_Attitude);
    Attitude_hub_DataPipe.data_size = DataPipe_DataSize(Hub_Attitude);
    Attitude_hub_DataPipe.trans_finish_cb = To_Pipe_TransFinish_Callback(SrvDataHub_Attitude_DataPipe_Finish_Callback);
    DataPipe_Enable(&Attitude_hub_DataPipe);

    memset(DataPipe_DataObjAddr(Hub_Baro_Data), 0, DataPipe_DataSize(Hub_Baro_Data));
    Baro_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Hub_Baro_Data);
    Baro_hub_DataPipe.data_size = DataPipe_DataSize(Hub_Baro_Data);
    Baro_hub_DataPipe.trans_finish_cb = To_Pipe_TransFinish_Callback(SrvDataHub_Baro_DataPipe_Finish_Callback);
    DataPipe_Enable(&Baro_hub_DataPipe);

    memset(DataPipe_DataObjAddr(Hub_Alt), 0, DataPipe_DataSize(Hub_Alt));
    Altitude_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Hub_Alt);
    Altitude_hub_DataPipe.data_size = DataPipe_DataSize(Hub_Alt);
    Altitude_hub_DataPipe.trans_finish_cb = To_Pipe_TransFinish_Callback(SrvDatHub_Alt_DataPipe_Finish_Callback);
    DataPipe_Enable(&Altitude_hub_DataPipe);

    memset(DataPipe_DataObjAddr(Hub_VCP_Attach_State), 0, DataPipe_DataSize(Hub_VCP_Attach_State));
    VCP_Connect_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Hub_VCP_Attach_State);
    VCP_Connect_hub_DataPipe.data_size = DataPipe_DataSize(Hub_VCP_Attach_State);
    VCP_Connect_hub_DataPipe.trans_finish_cb = To_Pipe_TransFinish_Callback(SrvDataHub_VCPAttach_dataPipe_Finish_Callback);
    DataPipe_Enable(&VCP_Connect_hub_DataPipe);
    
    memset(DataPipe_DataObjAddr(Hub_Cnv_CtlData), 0, DataPipe_DataSize(Hub_Cnv_CtlData));
    CtlData_hub_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Hub_Cnv_CtlData);
    CtlData_hub_DataPipe.data_size = DataPipe_DataSize(Hub_Cnv_CtlData);
    CtlData_hub_DataPipe.trans_finish_cb = To_Pipe_TransFinish_Callback(SrvDataHub_PipeConvertControlDataFinish_Callback);
    DataPipe_Enable(&CtlData_hub_DataPipe);

    memset(&SrvDataHub_Monitor, 0, sizeof(SrvDataHub_Monitor));
    SrvDataHub_Monitor.init_state = true;
    SrvDataHub_Monitor.data.failsafe = true;
    SrvDataHub_Monitor.data.arm = DRONE_ARM;
}

static void SrvDataHub_VCPAttach_dataPipe_Finish_Callback(DataPipeObj_TypeDef *obj)
{
    if (obj == &VCP_Connect_hub_DataPipe)
    {
        SrvDataHub_Monitor.update_reg.bit.USB_VCP_attach = true;

        if (SrvDataHub_Monitor.inuse_reg.bit.USB_VCP_attach)
            SrvDataHub_Monitor.inuse_reg.bit.USB_VCP_attach = false;

        SrvDataHub_Monitor.data.VCP_Attach = DataPipe_DataObj(Hub_VCP_Attach_State);

        SrvDataHub_Monitor.update_reg.bit.USB_VCP_attach = false;
    }
}

static void SrvDatHub_Alt_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj)
{
    if (obj == &Altitude_hub_DataPipe)
    {
        SrvDataHub_Monitor.update_reg.bit.relative_alt = true;

        if (SrvDataHub_Monitor.inuse_reg.bit.relative_alt)
            SrvDataHub_Monitor.inuse_reg.bit.relative_alt = false;
            
        SrvDataHub_Monitor.data.relative_alt_time = DataPipe_DataObj(Hub_Alt).time;
        SrvDataHub_Monitor.data.relative_alt = DataPipe_DataObj(Hub_Alt).alt;

        SrvDataHub_Monitor.update_reg.bit.relative_alt = false;
    }
}

static void SrvDataHub_Baro_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj)
{
    if(obj == &Baro_hub_DataPipe)
    {
        SrvDataHub_Monitor.update_reg.bit.scaled_baro = true;
    
        if(SrvDataHub_Monitor.inuse_reg.bit.scaled_baro)
            SrvDataHub_Monitor.inuse_reg.bit.scaled_baro = false;

        SrvDataHub_Monitor.data.baro_update_time = DataPipe_DataObj(Hub_Baro_Data).data.time_stamp;
        SrvDataHub_Monitor.data.baro_alt = DataPipe_DataObj(Hub_Baro_Data).data.pressure_alt;
        SrvDataHub_Monitor.data.baro_alt_offset = DataPipe_DataObj(Hub_Baro_Data).data.pressure_alt_offset;
        SrvDataHub_Monitor.data.baro_pressure = DataPipe_DataObj(Hub_Baro_Data).data.pressure;
        SrvDataHub_Monitor.data.baro_tempra = DataPipe_DataObj(Hub_Baro_Data).data.tempra;
        SrvDataHub_Monitor.data.baro_error_code = DataPipe_DataObj(Hub_Baro_Data).data.error_code;

        SrvDataHub_Monitor.update_reg.bit.scaled_baro = false;
    }
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
        SrvDataHub_Monitor.data.att_flip_over = DataPipe_DataObj(Hub_Attitude).flip_over;
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

static void SrvDataHub_IMU_Range_DataPipe_Finish_Callback(DataPipeObj_TypeDef *obj)
{
    if(obj == &IMU_PriRange_hub_DataPipe)
    {
        SrvDataHub_Monitor.update_reg.bit.range_imu = true;
        if(SrvDataHub_Monitor.inuse_reg.bit.range_imu)
            SrvDataHub_Monitor.inuse_reg.bit.range_imu = false;

        SrvDataHub_Monitor.data.pri_acc_range = DataPipe_DataObj(Hub_PriIMU_Range).Acc;
        SrvDataHub_Monitor.data.pri_gyr_range = DataPipe_DataObj(Hub_PriIMU_Range).Gyr;

        SrvDataHub_Monitor.update_reg.bit.range_imu = false;
    }
#if (IMU_CNT == 2)
    else if(obj == &IMU_SecRange_hub_DataPipe)
    {
        SrvDataHub_Monitor.update_reg.bit.range_imu = true;
        if(SrvDataHub_Monitor.inuse_reg.bit.range_imu)
            SrvDataHub_Monitor.inuse_reg.bit.range_imu = false;

        SrvDataHub_Monitor.data.sec_acc_range = DataPipe_DataObj(Hub_SecIMU_Range).Acc;
        SrvDataHub_Monitor.data.sec_gyr_range = DataPipe_DataObj(Hub_SecIMU_Range).Gyr;
    
        SrvDataHub_Monitor.update_reg.bit.range_imu = false;
    }
#endif
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
        SrvDataHub_Monitor.update_reg.bit.baro_init = true;

        if (SrvDataHub_Monitor.inuse_reg.bit.imu_init)
            SrvDataHub_Monitor.inuse_reg.bit.imu_init = false;

        if (SrvDataHub_Monitor.inuse_reg.bit.mag_init)
            SrvDataHub_Monitor.inuse_reg.bit.mag_init = false;

        if (SrvDataHub_Monitor.inuse_reg.bit.baro_init)
            SrvDataHub_Monitor.inuse_reg.bit.baro_init = false;

        if(obj == &SensorEnableState_hub_DataPipe)
        {
            SrvDataHub_Monitor.data.mag_enabled = DataPipe_DataObj(Sensor_Enable).bit.mag;
            SrvDataHub_Monitor.data.baro_enabled = DataPipe_DataObj(Sensor_Enable).bit.baro;
            SrvDataHub_Monitor.data.tof_enabled = DataPipe_DataObj(Sensor_Enable).bit.tof;
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
        SrvDataHub_Monitor.update_reg.bit.baro_init = false;
    }
}

static void SrvDataHub_PipeRcTelemtryDataFinish_Callback(DataPipeObj_TypeDef *obj)
{
    if ((!SrvDataHub_Monitor.init_state) || (obj == NULL))
        return;

    if (obj == &Receiver_hub_DataPipe)
    {
        SrvDataHub_Monitor.update_reg.bit.rc_control_data = true;

        if (SrvDataHub_Monitor.inuse_reg.bit.rc_control_data)
            SrvDataHub_Monitor.inuse_reg.bit.rc_control_data = false;

        SrvDataHub_Monitor.data.RC_Control_Data = DataPipe_DataObj(Hub_Telemetry_Rc);

        SrvDataHub_Monitor.update_reg.bit.rc_control_data = false;
    }
}

static void SrvDataHub_PipeConvertControlDataFinish_Callback(DataPipeObj_TypeDef *obj)
{
    if ((!SrvDataHub_Monitor.init_state) || (obj == NULL))
        return;

    if (obj == &CtlData_hub_DataPipe)
    {
        SrvDataHub_Monitor.update_reg.bit.cnv_control_data = true;

        if (SrvDataHub_Monitor.inuse_reg.bit.cnv_control_data)
            SrvDataHub_Monitor.inuse_reg.bit.cnv_control_data = false;

        SrvDataHub_Monitor.data.cnvctl_data_time = SrvOsCommon.get_os_ms();
        SrvDataHub_Monitor.data.arm = DataPipe_DataObj(Hub_Cnv_CtlData).arm;
        SrvDataHub_Monitor.data.failsafe = DataPipe_DataObj(Hub_Cnv_CtlData).failsafe;
        SrvDataHub_Monitor.data.ctl_mode = DataPipe_DataObj(Hub_Cnv_CtlData).control_mode;
        SrvDataHub_Monitor.data.exp_gyr_x = DataPipe_DataObj(Hub_Cnv_CtlData).gyr_x;
        SrvDataHub_Monitor.data.exp_gyr_y = DataPipe_DataObj(Hub_Cnv_CtlData).gyr_y;
        SrvDataHub_Monitor.data.exp_gyr_z = DataPipe_DataObj(Hub_Cnv_CtlData).gyr_z;
        SrvDataHub_Monitor.data.exp_pitch = DataPipe_DataObj(Hub_Cnv_CtlData).pitch;
        SrvDataHub_Monitor.data.exp_roll = DataPipe_DataObj(Hub_Cnv_CtlData).roll;

        SrvDataHub_Monitor.update_reg.bit.cnv_control_data = false;
    }
}

static bool SrvDataHub_Get_PriIMU_Range(uint8_t *acc_range, uint16_t *gyr_range)
{
    if(acc_range && gyr_range)
    {
reupdate_pri_imu_range:
        SrvDataHub_Monitor.inuse_reg.bit.range_imu = true;

        (*acc_range) = SrvDataHub_Monitor.data.pri_acc_range;
        (*gyr_range) = SrvDataHub_Monitor.data.pri_gyr_range;

        if(!SrvDataHub_Monitor.inuse_reg.bit.range_imu)
            goto reupdate_pri_imu_range;

        SrvDataHub_Monitor.inuse_reg.bit.range_imu = false;

        return true;
    }

    return false;
}

static bool SrvDataHub_Get_SecIMU_Range(uint8_t *acc_range, uint16_t *gyr_range)
{
    if(acc_range && gyr_range)
    {
        (*acc_range) = SrvDataHub_Monitor.data.sec_acc_range;
        (*gyr_range) = SrvDataHub_Monitor.data.sec_gyr_range;
    }

    return true; 
}

static bool SrvDataHub_Get_Raw_IMU(uint32_t *time_stamp, float *acc_scale, float *gyr_scale, float *acc_x, float *acc_y, float *acc_z, float *gyr_x, float *gyr_y, float *gyr_z, float *tmpr, uint8_t *err)
{
reupdate_raw_imu:
    SrvDataHub_Monitor.inuse_reg.bit.raw_imu = true;

    if (time_stamp)
        *time_stamp = SrvDataHub_Monitor.data.imu_update_time;
    
    if (acc_scale)
        *acc_scale = SrvDataHub_Monitor.data.acc_scale;
    
    if (gyr_scale)
        *gyr_scale = SrvDataHub_Monitor.data.gyr_scale;
    
    if (acc_x)
        *acc_x = SrvDataHub_Monitor.data.org_acc_x;
    
    if (acc_y)
        *acc_y = SrvDataHub_Monitor.data.org_acc_y;
    
    if (acc_z)
        *acc_z = SrvDataHub_Monitor.data.org_acc_z;
    
    if (gyr_x)
        *gyr_x = SrvDataHub_Monitor.data.org_gyr_x;
    
    if (gyr_y)
        *gyr_y = SrvDataHub_Monitor.data.org_gyr_y;
    
    if (gyr_z)
        *gyr_z = SrvDataHub_Monitor.data.org_gyr_z;
    
    if (tmpr)
        *tmpr = SrvDataHub_Monitor.data.imu_temp;
    
    if (err)
        *err = SrvDataHub_Monitor.data.imu_error_code;

    if (!SrvDataHub_Monitor.inuse_reg.bit.raw_imu)
        goto reupdate_raw_imu;

    SrvDataHub_Monitor.inuse_reg.bit.raw_imu = false;

    return true;
}

static bool SrvDataHub_Get_Scaled_IMU(uint32_t *time_stamp, float *acc_scale, float *gyr_scale, float *acc_x, float *acc_y, float *acc_z, float *gyr_x, float *gyr_y, float *gyr_z, float *tmpr, uint8_t *err)
{
reupdate_scaled_imu:
    SrvDataHub_Monitor.inuse_reg.bit.scaled_imu = true;

    if (time_stamp)
        *time_stamp = SrvDataHub_Monitor.data.imu_update_time; 
    
    if (acc_scale)
        *acc_scale = SrvDataHub_Monitor.data.acc_scale;
    
    if (gyr_scale)
        *gyr_scale = SrvDataHub_Monitor.data.gyr_scale;
    
    if (acc_x)
        *acc_x = SrvDataHub_Monitor.data.flt_acc_x;
    
    if (acc_y)
        *acc_y = SrvDataHub_Monitor.data.flt_acc_y;
    
    if (acc_z)
        *acc_z = SrvDataHub_Monitor.data.flt_acc_z;
    
    if (gyr_x)
        *gyr_x = SrvDataHub_Monitor.data.flt_gyr_x;
    
    if (gyr_y)
        *gyr_y = SrvDataHub_Monitor.data.flt_gyr_y;
    
    if (gyr_z)
        *gyr_z = SrvDataHub_Monitor.data.flt_gyr_z;
    
    if (tmpr)
        *tmpr = SrvDataHub_Monitor.data.imu_temp;
    
    if (err)
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

    SrvDataHub_Monitor.inuse_reg.bit.raw_mag = false;
    
    return true;
}

static bool SrvDataHub_Get_Scaled_Baro(uint32_t *time_stamp, float *baro_pressure, float *baro_alt, float *baro_alt_offset, float *tempra, uint8_t *error)
{
reupdate_scaled_baro:
    SrvDataHub_Monitor.inuse_reg.bit.scaled_baro = true;

    if (time_stamp)
        *time_stamp = SrvDataHub_Monitor.data.baro_update_time;
    
    if (baro_pressure)
        *baro_pressure = SrvDataHub_Monitor.data.baro_pressure;
    
    if (baro_alt)
        *baro_alt = SrvDataHub_Monitor.data.baro_alt;
    
    if (baro_alt_offset)
        *baro_alt_offset = SrvDataHub_Monitor.data.baro_alt_offset;
    
    if (tempra)
        *tempra = SrvDataHub_Monitor.data.baro_tempra;
    
    if (error)
        *error = SrvDataHub_Monitor.data.baro_error_code;

    if(!SrvDataHub_Monitor.inuse_reg.bit.scaled_baro)
        goto reupdate_scaled_baro;

    SrvDataHub_Monitor.inuse_reg.bit.scaled_baro = false;
    
    return true;
}

static bool SrvDataHub_Get_RelativeAlt(uint32_t *time_stamp, float *alt)
{
    SrvDataHub_Monitor.inuse_reg.bit.relative_alt = true;

reupdate_relative_alt:
    if (time_stamp)
        *time_stamp = SrvDataHub_Monitor.data.relative_alt_time;
    
    if (alt)
        *alt = SrvDataHub_Monitor.data.relative_alt;

    if (!SrvDataHub_Monitor.inuse_reg.bit.relative_alt)
        goto reupdate_relative_alt;

    SrvDataHub_Monitor.inuse_reg.bit.relative_alt = false;

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

    SrvDataHub_Monitor.inuse_reg.bit.scaled_mag = false;

    return true;
}

static bool SrvDataHub_Get_Attitude(uint32_t *time_stamp, float *pitch, float *roll, float *yaw, float *q0, float *q1, float *q2, float *q3, bool *flip_over)
{
reupdate_attitude:
    SrvDataHub_Monitor.inuse_reg.bit.attitude = true; 
    
    if (time_stamp)
        (*time_stamp) = SrvDataHub_Monitor.data.att_update_time;
    
    if (pitch)
        (*pitch) = SrvDataHub_Monitor.data.att_pitch;
    
    if (roll)
        (*roll) = SrvDataHub_Monitor.data.att_roll;
    
    if (yaw)
        (*yaw) = SrvDataHub_Monitor.data.att_yaw;

    if (q0)
        (*q0) = SrvDataHub_Monitor.data.att_q0;

    if (q1)
        (*q1) = SrvDataHub_Monitor.data.att_q1;

    if (q2)
        (*q2) = SrvDataHub_Monitor.data.att_q2;
    
    if (q3)
        (*q3) = SrvDataHub_Monitor.data.att_q3;

    if (flip_over)
        (*flip_over) = SrvDataHub_Monitor.data.att_flip_over;

    if(!SrvDataHub_Monitor.inuse_reg.bit.attitude)
        goto reupdate_attitude;

    SrvDataHub_Monitor.inuse_reg.bit.attitude = false;

    return true;
}

static bool SrvDataHub_Get_Arm(bool *arm)
{
    if (arm == NULL)
        return false;

reupdate_arm:
    SrvDataHub_Monitor.inuse_reg.bit.cnv_control_data = true;
    *arm = SrvDataHub_Monitor.data.arm;

    if (!SrvDataHub_Monitor.inuse_reg.bit.cnv_control_data)
        goto reupdate_arm;

    SrvDataHub_Monitor.inuse_reg.bit.cnv_control_data = false;

    return true;
}

static bool SrvDataHub_Get_Failsafe(bool *failsafe)
{
reupdate_failsafe:
    SrvDataHub_Monitor.inuse_reg.bit.cnv_control_data = true;
    
    if (failsafe)
        *failsafe = SrvDataHub_Monitor.data.failsafe;

    if (!SrvDataHub_Monitor.inuse_reg.bit.cnv_control_data)
        goto reupdate_failsafe;

    SrvDataHub_Monitor.inuse_reg.bit.cnv_control_data = false;

    return true;
}

static bool SrvDataHub_Get_Convert_ControlData(uint32_t *time_stamp, bool *arm, bool *failsafe, uint8_t *mode, float *pitch, float *roll, float *gx, float *gy, float *gz)
{
    if ((time_stamp == NULL) || \
        (arm == NULL) || \
        (failsafe == NULL) || \
        (mode == NULL) || \
        (pitch == NULL) || \
        (roll == NULL) || \
        (gx == NULL) || \
        (gy == NULL) || \
        (gz == NULL))
        return false;

reupdate_convertdata:
    SrvDataHub_Monitor.inuse_reg.bit.cnv_control_data = true;

    *time_stamp = SrvDataHub_Monitor.data.cnvctl_data_time;
    *arm = SrvDataHub_Monitor.data.arm;
    *failsafe = SrvDataHub_Monitor.data.failsafe;
    *mode = SrvDataHub_Monitor.data.ctl_mode;
    *pitch = SrvDataHub_Monitor.data.exp_pitch;
    *roll = SrvDataHub_Monitor.data.exp_roll;
    *gx = SrvDataHub_Monitor.data.exp_gyr_x;
    *gy = SrvDataHub_Monitor.data.exp_gyr_y;
    *gz = SrvDataHub_Monitor.data.exp_gyr_z;

    if (!SrvDataHub_Monitor.inuse_reg.bit.cnv_control_data)
        goto reupdate_convertdata;

    SrvDataHub_Monitor.inuse_reg.bit.cnv_control_data = false;
    
    return true;
}

static bool SrvDataHub_Get_Telemetry_ControlData(ControlData_TypeDef *data)
{
    if(data)
    {
reupdate_telemetry_control_data:
        SrvDataHub_Monitor.inuse_reg.bit.rc_control_data = true;

        (*data) = SrvDataHub_Monitor.data.RC_Control_Data;

        if(!SrvDataHub_Monitor.inuse_reg.bit.rc_control_data)
            goto reupdate_telemetry_control_data;

        SrvDataHub_Monitor.inuse_reg.bit.rc_control_data = false;

        return true;
    }

    return false;
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
        *state = SrvDataHub_Monitor.data.imu_init_state;

    if(!SrvDataHub_Monitor.inuse_reg.bit.imu_init)
        goto reupdate_imu_state;

    SrvDataHub_Monitor.inuse_reg.bit.imu_init = false;

    return true;
}

static bool SrvDataHub_Get_Mag_InitState(bool *state)
{
reupdate_mag_state:
    SrvDataHub_Monitor.inuse_reg.bit.mag_init = true;

    if(state)
        *state = SrvDataHub_Monitor.data.mag_init_state;

    if(!SrvDataHub_Monitor.inuse_reg.bit.mag_init)
        goto reupdate_mag_state;

    SrvDataHub_Monitor.inuse_reg.bit.mag_init = false;

    return true;
}

static bool SrvDataHub_Get_Bar_InitState(bool *state)
{
reupdate_baro_state:
    SrvDataHub_Monitor.inuse_reg.bit.baro_init = true;

    if (state)
        *state = SrvDataHub_Monitor.data.baro_init_state;

    if (!SrvDataHub_Monitor.inuse_reg.bit.baro_init)
        goto reupdate_baro_state;

    SrvDataHub_Monitor.inuse_reg.bit.baro_init = false;

    return true;
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
    }

    if (!SrvDataHub_Monitor.inuse_reg.bit.actuator)
        goto reupdate_servo_channel;

    SrvDataHub_Monitor.inuse_reg.bit.actuator = false;

    return true;
}

static bool SrvDataHub_Set_CLI_State(bool state)
{
    if(SrvDataHub_Monitor.inuse_reg.bit.cli)
        SrvDataHub_Monitor.inuse_reg.bit.cli = false;

    SrvDataHub_Monitor.update_reg.bit.cli = true;
    SrvDataHub_Monitor.data.CLI_state = state;
    SrvDataHub_Monitor.update_reg.bit.cli = false;
    
    return true;
}

static bool SrvDataHub_Get_CLI_State(bool *state)
{
    if(state)
    {
reupdate_cli_state:
        SrvDataHub_Monitor.inuse_reg.bit.cli = true;

        (*state) = SrvDataHub_Monitor.data.CLI_state;

        if(!SrvDataHub_Monitor.inuse_reg.bit.cli)
            goto reupdate_cli_state;

        SrvDataHub_Monitor.inuse_reg.bit.cli = false;

        return true;
    }

    return false;
}

static bool SrvDataHub_Set_Upgrade_State(bool state)
{
    if (SrvDataHub_Monitor.inuse_reg.bit.upgrade)
        SrvDataHub_Monitor.inuse_reg.bit.upgrade = false;

    SrvDataHub_Monitor.update_reg.bit.upgrade = true;
    SrvDataHub_Monitor.data.upgrading = state;
    SrvDataHub_Monitor.update_reg.bit.upgrade = false;
    
    return true; 
}

static bool SrvDataHub_Get_Upgrade_State(bool *state)
{
    if(state)
    {
reupdate_upgrade_state:
        SrvDataHub_Monitor.inuse_reg.bit.upgrade = true;

        (*state) = SrvDataHub_Monitor.data.upgrading;

        if(!SrvDataHub_Monitor.inuse_reg.bit.upgrade)
            goto reupdate_upgrade_state;

        SrvDataHub_Monitor.inuse_reg.bit.upgrade = false;
        
        return true;
    }

    return false;
}

static bool SrvDataHub_Get_VCPAttach_State(bool *state)
{
    if(state)
    {
reupdate_vcp_attach_state:
        SrvDataHub_Monitor.inuse_reg.bit.USB_VCP_attach = true;

        (*state) = SrvDataHub_Monitor.data.VCP_Attach;

        if(!SrvDataHub_Monitor.inuse_reg.bit.USB_VCP_attach)
            goto reupdate_vcp_attach_state;

        SrvDataHub_Monitor.inuse_reg.bit.USB_VCP_attach = false;
        
        return true;
    }

    return false;
}

