/*
 *  Auther : 8_B!T0
 *  this file use for moto & servo control
 *  (but currently none servo)
 *  pitch & roll control mode outer loop is angular loop inner loop is angular speed loop 
 *  yaw control mode is only angular speed loop
 */
#include "Task_Control.h"
#include "Srv_OsCommon.h"
#include "Task_Telemetry.h"
#include "Srv_DataHub.h"
#include "Srv_Actuator.h"
#include "shell_port.h"

#define CONTROL_STORAGE_SECTION_NAME "Control_Para"

#define ATTITUDE_PID_ACCURACY 1000
#define ATTITUDE_PID_DIFF_MAX 30    /* unit: deg */
#define ATTITUDE_PID_DIFF_MIN -30   /* unit: deg */

#define ANGULAR_PID_ACCURACY 1000
#define THROTTLE_CHANGE_RATE 50   /* unit value per ms */

#define ANGULAR_RATE_PID_ACCURACY 1000
#define GYRO_X_RATE_PID_DIFF_MAX 100
#define GYRO_X_RATE_PID_DIFF_MIN -100
#define GYRO_Y_RATE_PID_DIFF_MAX 100
#define GYRO_Y_RATE_PID_DIFF_MIN -100
#define GYRO_Z_RATE_PID_DIFF_MAX 50
#define GYRO_Z_RATE_PID_DIFF_MIN -50

#define CONTROL_ATT_RANGE_MAX 30.0f
#define CONTROL_ATT_RANGE_MID 0.0f
#define CONTROL_ATT_RANGE_MIN -30.0f

#define CONTROL_GYR_RANGE_MAX 800.0f
#define CONTROL_GYR_RANGE_MID 0.0f
#define CONTROL_GYR_RANGE_MIN -800.0f

static uint32_t imu_update_time = 0;
static uint32_t att_update_time = 0;
static uint8_t imu_err_code;
static bool arm_state = true;
static bool failsafe = false;
static bool imu_init_state = false;
static bool att_update = false;
static bool configrator_attach = false;

SrvIMU_UnionData_TypeDef LstCyc_IMU_Data;
SrvRecever_RCSig_TypeDef LstCyc_Rc_Data;

DataPipe_CreateDataObj(ExpControlData_TypeDef, ExpCtl);

TaskControl_Monitor_TypeDef TaskControl_Monitor = {
    .init_state = false,
    .control_abort = false,

    /* on test mode use angular_speed over rate threshold protect */
    .angular_protect_enable = true,
    .angular_protect = false,

    /* on test mode for throttle control value mutation protect */
    .throttle_protect_enable = true,
    .throttle_percent = false,

    .IMU_Rt = 0,
};

/* internal function */
static bool TaskControl_AttitudeRing_PID_Update(TaskControl_Monitor_TypeDef *monitor);
static bool TaskControl_AngularSpeedRing_PID_Update(TaskControl_Monitor_TypeDef *monitor);
static void TaskControl_FlightControl_Polling(ControlData_TypeDef *exp_ctl_val);
static void TaskControl_Actuator_ControlValue_Update(TaskControl_Monitor_TypeDef *monitor);
static void TaskControl_CLI_Polling(void);
static void TaskControl_Get_StoreParam(void);
static TaskControl_FlightParam_TypeDef TaskControl_Get_InuseParam(void);

/* internal var */
static uint32_t TaskControl_Period = 0;

void TaskControl_Init(uint32_t period)
{
    /* init monitor */
    memset(&TaskControl_Monitor, 0, sizeof(TaskControl_Monitor));

    /* pipe init */
    CtlData_smp_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(ExpCtl);
    CtlData_smp_DataPipe.data_size = DataPipe_DataSize(ExpCtl);
    DataPipe_Enable(&CtlData_smp_DataPipe);

    /* Parametet Init */
    TaskControl_Get_StoreParam();

    TaskControl_Monitor.init_state = SrvActuator.init(TaskControl_Monitor.actuator_param);
    SrvDataHub.get_imu_init_state(&imu_init_state);

    osMessageQDef(MotoCLI_Data, 64, TaskControl_CLIData_TypeDef);
    TaskControl_Monitor.CLIMessage_ID = osMessageCreate(osMessageQ(MotoCLI_Data), NULL);
    
    TaskControl_Period = period;
}

static TaskControl_FlightParam_TypeDef TaskControl_Get_DefaultParam(void)
{
    TaskControl_FlightParam_TypeDef Param;

    memset(&Param, 0, sizeof(TaskControl_FlightParam_TypeDef));

    /* use defaule range data */
    Param.pitch.max = CONTROL_ATT_RANGE_MAX;
    Param.pitch.mid = CONTROL_ATT_RANGE_MID;
    Param.pitch.min = CONTROL_ATT_RANGE_MIN;

    Param.roll.max  = CONTROL_ATT_RANGE_MAX;
    Param.roll.mid  = CONTROL_ATT_RANGE_MID;
    Param.roll.min  = CONTROL_ATT_RANGE_MIN;

    Param.gx.max    = CONTROL_GYR_RANGE_MAX;
    Param.gx.mid    = CONTROL_GYR_RANGE_MID;
    Param.gx.min    = CONTROL_GYR_RANGE_MIN;

    Param.gy.max    = CONTROL_GYR_RANGE_MAX;
    Param.gy.mid    = CONTROL_GYR_RANGE_MID;
    Param.gy.min    = CONTROL_GYR_RANGE_MIN;

    Param.gz.max    = CONTROL_GYR_RANGE_MAX;
    Param.gz.mid    = CONTROL_GYR_RANGE_MID;
    Param.gz.min    = CONTROL_GYR_RANGE_MIN;

    /* use default pid data */
    /* attitude PID control parameter section */
    Param.Outer.Pitch_Para.gP_Diff_Max = ATTITUDE_PID_DIFF_MAX;
    Param.Outer.Pitch_Para.gP_Diff_Min = ATTITUDE_PID_DIFF_MIN;
    Param.Outer.Pitch_Para.gP          = 1.2;
    Param.Outer.Pitch_Para.gI          = 0.08;
    Param.Outer.Pitch_Para.gI_Max      = 50;
    Param.Outer.Pitch_Para.gI_Min      = -50;
    Param.Outer.Pitch_Para.gD          = 1;

    Param.Outer.Roll_Para.gP_Diff_Max  = ATTITUDE_PID_DIFF_MAX;
    Param.Outer.Roll_Para.gP_Diff_Min  = ATTITUDE_PID_DIFF_MIN;
    Param.Outer.Roll_Para.gP           = 1.2;
    Param.Outer.Roll_Para.gI           = 0.08;
    Param.Outer.Roll_Para.gI_Max       = 50;
    Param.Outer.Roll_Para.gI_Min       = -50;
    Param.Outer.Roll_Para.gD           = 1;

    /* angular PID control parameter section */
    Param.Inner.GyroX_Para.gP_Diff_Max = GYRO_X_RATE_PID_DIFF_MAX;
    Param.Inner.GyroX_Para.gP_Diff_Min = GYRO_X_RATE_PID_DIFF_MIN;
    Param.Inner.GyroX_Para.gP          = 1.2;
    Param.Inner.GyroX_Para.gI          = 0.002;
    Param.Inner.GyroX_Para.gI_Max      = 30;
    Param.Inner.GyroX_Para.gI_Min      = -30;
    Param.Inner.GyroX_Para.gD          = 0.1;

    Param.Inner.GyroY_Para.gP_Diff_Max = GYRO_Y_RATE_PID_DIFF_MAX;
    Param.Inner.GyroY_Para.gP_Diff_Min = GYRO_Y_RATE_PID_DIFF_MIN;
    Param.Inner.GyroY_Para.gP          = 1.2;
    Param.Inner.GyroY_Para.gI          = 0.002;
    Param.Inner.GyroY_Para.gI_Max      = 30;
    Param.Inner.GyroY_Para.gI_Min      = -30;
    Param.Inner.GyroY_Para.gD          = 0.1;

    Param.Inner.GyroZ_Para.gP_Diff_Max = GYRO_X_RATE_PID_DIFF_MAX;
    Param.Inner.GyroZ_Para.gP_Diff_Min = GYRO_X_RATE_PID_DIFF_MIN;
    Param.Inner.GyroZ_Para.gP          = 1;
    Param.Inner.GyroZ_Para.gI          = 0;
    Param.Inner.GyroZ_Para.gI_Max      = 0;
    Param.Inner.GyroZ_Para.gI_Min      = 0;
    Param.Inner.GyroZ_Para.gD          = 0.1;

    return Param;
}

static bool TaskControl_Param_Copy(PIDObj_TypeDef *PID_Obj, PID_Param_TypeDef Para)
{
    if (PID_Obj)
    {
        PID_Obj->accuracy_scale = ANGULAR_PID_ACCURACY;
        PID_Obj->gP             = Para.gP;
        PID_Obj->diff_max       = Para.gP_Diff_Max;
        PID_Obj->diff_min       = Para.gP_Diff_Min;
        PID_Obj->gI             = Para.gI;
        PID_Obj->gI_Max         = Para.gI_Max;
        PID_Obj->gI_Min         = Para.gI_Min;
        PID_Obj->gD             = Para.gD;

        return true;
    }

    return false;
}

/* read param from storage */
static void TaskControl_Get_StoreParam(void)
{
    TaskControl_FlightParam_TypeDef PID_Param;
    TaskControl_FlightParam_TypeDef Default_Param;
    TaskControl_FlightParam_TypeDef *p_UseParam = NULL;
    SrvActuator_Setting_TypeDef Actuator_Param;

    /* search storage section first */
    memset(&PID_Param, 0, sizeof(TaskControl_FlightParam_TypeDef));
    memset(&Actuator_Param, 0, sizeof(SrvActuator_Setting_TypeDef));
    memset(&Default_Param, 0, sizeof(TaskControl_FlightParam_TypeDef));
    memset(&TaskControl_Monitor.pid_store_info, 0, sizeof(Storage_ItemSearchOut_TypeDef));
    memset(&TaskControl_Monitor.actuator_store_info, 0, sizeof(Storage_ItemSearchOut_TypeDef));

    TaskControl_Monitor.pid_store_info = Storage.search(Para_User, CONTROL_STORAGE_SECTION_NAME);
    TaskControl_Monitor.actuator_store_info = Storage.search(Para_User, ACTUATOR_STORAGE_SECTION_NAME);

    /* get pid parameter */
    PID_Param = TaskControl_Get_DefaultParam();
    Default_Param = PID_Param;
    p_UseParam = &Default_Param;
    
    if (TaskControl_Monitor.pid_store_info.item_addr == 0)
    {
        /* no pid parameter found in external flash chip under user partten */
        /* section create successful */
        Storage.create(Para_User, CONTROL_STORAGE_SECTION_NAME, (uint8_t *)&PID_Param, sizeof(TaskControl_FlightParam_TypeDef));
    }
    else
    {
        if ((TaskControl_Monitor.pid_store_info.item.len == sizeof(TaskControl_FlightParam_TypeDef)) && \
            (Storage.get(Para_User, TaskControl_Monitor.pid_store_info.item, (uint8_t *)&PID_Param, sizeof(TaskControl_FlightParam_TypeDef)) == Storage_Error_None))
            p_UseParam = &PID_Param;
    }
    
    memcpy(&TaskControl_Monitor.param, p_UseParam, sizeof(TaskControl_FlightParam_TypeDef));

    TaskControl_Param_Copy(&TaskControl_Monitor.PitchCtl_PIDObj, p_UseParam->Outer.Pitch_Para);
    TaskControl_Param_Copy(&TaskControl_Monitor.RollCtl_PIDObj,  p_UseParam->Outer.Roll_Para);
    TaskControl_Param_Copy(&TaskControl_Monitor.GyrXCtl_PIDObj,  p_UseParam->Inner.GyroX_Para);
    TaskControl_Param_Copy(&TaskControl_Monitor.GyrYCtl_PIDObj,  p_UseParam->Inner.GyroY_Para);
    TaskControl_Param_Copy(&TaskControl_Monitor.GyrZCtl_PIDObj,  p_UseParam->Inner.GyroZ_Para);

    /* get actuator parameter */
    /* set as default first */
    TaskControl_Monitor.actuator_param = SrvActuator.default_param();
    if (TaskControl_Monitor.actuator_store_info.item_addr)
    {
        if ((TaskControl_Monitor.actuator_store_info.item.len == sizeof(SrvActuator_Setting_TypeDef)) && \
            Storage.get(Para_User, TaskControl_Monitor.actuator_store_info.item, (uint8_t *)&Actuator_Param, sizeof(SrvActuator_Setting_TypeDef)) == Storage_Error_None)
            TaskControl_Monitor.actuator_param = Actuator_Param;
    }
}

void TaskControl_Core(void const *arg)
{
    uint32_t sys_time = SrvOsCommon.get_os_ms();
    bool upgrade_state = false;
    ControlData_TypeDef CtlData;
    bool USB_Attach = false;
    memset(&CtlData, 0, sizeof(ControlData_TypeDef));
    while(1)
    {
        /* get control data from data hub */
        SrvDataHub.get_rc_control_data(&CtlData);

        if (SrvDataHub.get_upgrade_state(&upgrade_state) && upgrade_state)
        {
            /* lock all actuator when upgrading */
            SrvActuator.lock();
        }
        else
        {
            if (!TaskControl_Monitor.CLI_enable)
            {
                /* lock moto when usb attached */
                if (!SrvDataHub.get_vcp_attach_state(&USB_Attach) || USB_Attach)
                {
                    SrvActuator.lock();
                }
                else
                    /* debug set control to angular speed control */
                    TaskControl_FlightControl_Polling(&CtlData);
            }
            else
            {
                if(TaskControl_Monitor.CLI_enable)
                {
                    TaskControl_CLI_Polling();
                }
                else
                    /* lock all moto */
                    SrvActuator.lock();
            }
        }

        SrvOsCommon.precise_delay(&sys_time, TaskControl_Period);
    }
}

static bool TaskControl_AttitudeRing_PID_Update(TaskControl_Monitor_TypeDef *monitor)
{
    if(monitor)
    {
        /* pitch PID update */
        PID_Update(&monitor->PitchCtl_PIDObj, monitor->attitude.pitch, monitor->exp_pitch);

        /* roll PID update */
        PID_Update(&monitor->RollCtl_PIDObj, monitor->attitude.roll, monitor->exp_roll);

        return true;
    }

    return false;
}

static bool TaskControl_AngularSpeedRing_PID_Update(TaskControl_Monitor_TypeDef *monitor)
{
    if(monitor)
    {
        /* gyro X PID Update */
        PID_Update(&monitor->GyrXCtl_PIDObj, monitor->gyr[Axis_X], monitor->GyrXCtl_PIDObj.exp);

        /* gyro Y PID Update */
        PID_Update(&monitor->GyrYCtl_PIDObj, monitor->gyr[Axis_Y], monitor->GyrYCtl_PIDObj.exp);

        /* gyro Z PID Update */
        PID_Update(&monitor->GyrZCtl_PIDObj, monitor->gyr[Axis_Z], monitor->GyrZCtl_PIDObj.exp);

        return true;
    }

    return false;
}

static void TaskControl_Actuator_ControlValue_Update(TaskControl_Monitor_TypeDef *monitor)
{
    uint16_t ctl_buf[Actuator_Ctl_Sum] = {0};

    if(monitor)
    {
        /* idle spin when disarm */
        ctl_buf[Actuator_Ctl_Throttle] = monitor->throttle_percent;

        ctl_buf[Actuator_Ctl_GyrX] = monitor->GyrXCtl_PIDObj.fout;
        ctl_buf[Actuator_Ctl_GyrY] = monitor->GyrYCtl_PIDObj.fout;
        ctl_buf[Actuator_Ctl_GyrZ] = monitor->GyrZCtl_PIDObj.fout;
    }

    SrvActuator.moto_control(ctl_buf);
}

/****************************************************** Flight Control Section ********************************************************/
static bool TaskControl_Convert_CtlData(TaskControl_Monitor_TypeDef *monitor, ControlData_TypeDef ctl_data)
{
    float p_scope = 0.0f;
    float n_scope = 0.0f;
    TaskControl_CtlRange_Para_TypeDef *p_range = NULL;

    /* convert pitch */
    p_range = &monitor->param.pitch;
    if ((p_range->max <= p_range->min) || (p_range->mid <= p_range->min) || (p_range->max <= p_range->mid))
        return false;

    p_scope = p_range->max - p_range->mid;
    n_scope = p_range->mid - p_range->min;

    monitor->exp_pitch = (ctl_data.pitch_percent - 50.0f) / 100.0f;
    if (monitor->exp_pitch >= 0)
    {
        monitor->exp_pitch *= p_scope;
    }
    else
        monitor->exp_pitch *= n_scope;

    /* convert roll */
    p_range = &monitor->param.roll;
    if ((p_range->max <= p_range->min) || (p_range->mid <= p_range->min) || (p_range->max <= p_range->mid))
        return false;

    p_scope = p_range->max - p_range->mid;
    n_scope = p_range->mid - p_range->min;

    monitor->exp_roll = (ctl_data.roll_percent - 50.0f) / 100.0f;
    if (monitor->exp_roll >= 0)
    {
        monitor->exp_roll *= p_scope;
    }
    else
        monitor->exp_roll *= n_scope;

    /* convert gyro x */
    p_range = &monitor->param.gx;
    if ((p_range->max <= p_range->min) || (p_range->mid <= p_range->min) || (p_range->max <= p_range->mid))
        return false;

    p_scope = p_range->max - p_range->mid;
    n_scope = p_range->mid - p_range->min;

    monitor->exp_gyr_x = (ctl_data.roll_percent - 50.0f) / 100.0f;
    if (monitor->exp_gyr_x >= 0)
    {
        monitor->exp_gyr_x *= p_scope;
    }
    else
        monitor->exp_gyr_x *= n_scope;

    /* convert gyro y */
    p_range = &monitor->param.gy;
    if ((p_range->max <= p_range->min) || (p_range->mid <= p_range->min) || (p_range->max <= p_range->mid))
        return false;

    p_scope = p_range->max - p_range->mid;
    n_scope = p_range->mid - p_range->min;

    monitor->exp_gyr_y = (ctl_data.pitch_percent - 50.0f) / 100.0f;
    if (monitor->exp_gyr_y >= 0)
    {
        monitor->exp_gyr_y *= p_scope;
    }
    else
        monitor->exp_gyr_y *= n_scope;

    /* convert gyro z */
    p_range = &monitor->param.gz;
    if ((p_range->max <= p_range->min) || (p_range->mid <= p_range->min) || (p_range->max <= p_range->mid))
        return false;

    p_scope = p_range->max - p_range->mid;
    n_scope = p_range->mid - p_range->min;

    monitor->exp_gyr_z = (ctl_data.yaw_percent - 50.0f) / 100.0f;
    if (monitor->exp_gyr_z >= 0)
    {
        monitor->exp_gyr_z *= p_scope;
    }
    else
        monitor->exp_gyr_z *= n_scope;

    return true;
}

/* need to be optmize */
static void TaskControl_FlightControl_Polling(ControlData_TypeDef *exp_ctl_val)
{
    uint8_t axis = Axis_X;
    uint32_t tunning_port = 0;
    bool arm_state = false;

    if (TaskControl_Monitor.init_state && exp_ctl_val)
    {
        TaskControl_Convert_CtlData(&TaskControl_Monitor, *exp_ctl_val);
        arm_state = exp_ctl_val->arm_state;
        failsafe = exp_ctl_val->fail_safe;
        TaskControl_Monitor.control_abort = false;

        /* pipe converted control data to data hub */
        DataPipe_DataObj(ExpCtl).arm = arm_state;
        DataPipe_DataObj(ExpCtl).failsafe = failsafe;
        DataPipe_DataObj(ExpCtl).control_mode = exp_ctl_val->control_mode;
        DataPipe_DataObj(ExpCtl).pitch = TaskControl_Monitor.exp_pitch;
        DataPipe_DataObj(ExpCtl).roll = TaskControl_Monitor.exp_roll;
        DataPipe_DataObj(ExpCtl).gyr_x = TaskControl_Monitor.exp_gyr_x;
        DataPipe_DataObj(ExpCtl).gyr_x = TaskControl_Monitor.exp_gyr_y;
        DataPipe_DataObj(ExpCtl).gyr_x = TaskControl_Monitor.exp_gyr_z;
        DataPipe_SendTo(&CtlData_smp_DataPipe, &CtlData_hub_DataPipe);
        DataPipe_SendTo(&CtlData_smp_DataPipe, &CtlData_Log_DataPipe);

        /* if armed or usb attached then lock moto */
        if ((arm_state == DRONE_ARM) || configrator_attach)
            goto lock_moto;

        // check imu filter gyro data update or not
        if(!SrvDataHub.get_scaled_imu(&imu_update_time,
                                      &TaskControl_Monitor.acc_scale,
                                      &TaskControl_Monitor.gyr_scale,
                                      &TaskControl_Monitor.acc[Axis_X],
                                      &TaskControl_Monitor.acc[Axis_Y],
                                      &TaskControl_Monitor.acc[Axis_Z],
                                      &TaskControl_Monitor.gyr[Axis_X],
                                      &TaskControl_Monitor.gyr[Axis_Y],
                                      &TaskControl_Monitor.gyr[Axis_Z],
                                      &TaskControl_Monitor.imu_tmpr,
                                      &imu_err_code) || !imu_init_state)
            goto lock_moto;

        /* if angular speed over ride then lock the moto and set drone as arm */

        // get attitude
        att_update = SrvDataHub.get_attitude(&att_update_time,
                                             &TaskControl_Monitor.attitude.pitch,
                                             &TaskControl_Monitor.attitude.roll,
                                             &TaskControl_Monitor.attitude.yaw,
                                             &TaskControl_Monitor.attitude.q0,
                                             &TaskControl_Monitor.attitude.q1,
                                             &TaskControl_Monitor.attitude.q2,
                                             &TaskControl_Monitor.attitude.q3,
                                             &TaskControl_Monitor.flip_over);

        if (imu_update_time)
        {
            if (imu_update_time > TaskControl_Monitor.IMU_Rt)
            {
                TaskControl_Monitor.imu_update_error_cnt = 0;
                TaskControl_Monitor.IMU_Rt = imu_update_time;
            }
            else if (imu_update_time <= TaskControl_Monitor.IMU_Rt)
            {
                TaskControl_Monitor.imu_update_error_cnt++;
                if (TaskControl_Monitor.imu_update_error_cnt >= IMU_ERROR_UPDATE_MAX_COUNT)
                {
                    TaskControl_Monitor.control_abort = true;
                    goto lock_moto;
                }
            }

            if(imu_err_code != SrvIMU_Sample_NoError)
            {
                switch(imu_err_code)
                {
                    case SrvIMU_Sample_Data_Acc_Blunt:
                    case SrvIMU_Sample_Data_Acc_OverRange:
                        /* still can use gyro loop control quad */
                        /* switch into manul control */
                        break;
                    
                    case SrvIMU_Sample_Data_Gyr_Blunt:
                    case SrvIMU_Sample_Data_Gyr_OverRange:
                        /* totally waste */
                        /* bye drone see u in another world */
                        TaskControl_Monitor.control_abort = true;
                        goto lock_moto;
                        break;

                    case SrvIMU_Sample_Module_UnReady:
                        TaskControl_Monitor.imu_none_update_cnt++;
                        if(TaskControl_Monitor.imu_none_update_cnt >= IMU_NONE_UPDATE_THRESHOLD)
                        {
                            TaskControl_Monitor.control_abort = true;
                            goto lock_moto;
                        }
                        else
                        {
                            /* use last time sample imu data for control */
                            for(axis = Axis_X; axis < Axis_Sum; axis++)
                            {
                                TaskControl_Monitor.acc[axis] = TaskControl_Monitor.acc_lst[axis];
                                TaskControl_Monitor.gyr[axis] = TaskControl_Monitor.gyr_lst[axis];
                            }
                        }
                        break;

                    case SrvIMU_Sample_Over_Angular_Accelerate:
                        if(TaskControl_Monitor.angular_warning_cnt < OVER_ANGULAR_ACCELERATE_COUNT)
                        {
                            TaskControl_Monitor.angular_warning_cnt++;

                            for(axis = Axis_X; axis < Axis_Sum; axis++)
                            {
                                TaskControl_Monitor.acc[axis] = TaskControl_Monitor.acc_lst[axis];
                                TaskControl_Monitor.gyr[axis] = TaskControl_Monitor.gyr_lst[axis];
                            }
                        }
                        else
                            TaskControl_Monitor.angular_protect = true;
                        break;
                }
            }
            else
                TaskControl_Monitor.imu_none_update_cnt = 0;
        }
        else
        {
            /* check keep time to abort drone control */
            TaskControl_Monitor.imu_none_update_cnt ++;

            if(TaskControl_Monitor.imu_none_update_cnt >= IMU_NONE_UPDATE_THRESHOLD)
                TaskControl_Monitor.control_abort = true;

            goto lock_moto;
        }
        
        if(TaskControl_Monitor.angular_protect_enable && TaskControl_Monitor.angular_protect)
            goto lock_moto;

        if(TaskControl_Monitor.flip_over)
        {
            if (!exp_ctl_val->aux.bit.flip_over)
                goto lock_moto;

            /* when drone is up side down and we want to flip over it by telemetry */
            /* reverse propeller spin dir to reverse the drone */
        }
        else
        {
            /* do drone control algorithm down below */
            TaskControl_Monitor.throttle_percent = exp_ctl_val->throttle_percent;

            /* Update PID */
            if(exp_ctl_val->control_mode == Attitude_Control)
            {
                /* attitude mode */
                /* set expection attitude */
                TaskControl_Monitor.RollCtl_PIDObj.exp = TaskControl_Monitor.exp_roll;
                TaskControl_Monitor.PitchCtl_PIDObj.exp = TaskControl_Monitor.exp_pitch;
                TaskControl_AttitudeRing_PID_Update(&TaskControl_Monitor);
                
                TaskControl_Monitor.GyrXCtl_PIDObj.exp = TaskControl_Monitor.RollCtl_PIDObj.fout;
                TaskControl_Monitor.GyrYCtl_PIDObj.exp = TaskControl_Monitor.PitchCtl_PIDObj.fout;
                TaskControl_Monitor.exp_gyr_x = TaskControl_Monitor.RollCtl_PIDObj.fout;
                TaskControl_Monitor.exp_gyr_y = TaskControl_Monitor.PitchCtl_PIDObj.fout;
            }
            else
            {
                /* angular speed mode */
                TaskControl_Monitor.GyrXCtl_PIDObj.exp = TaskControl_Monitor.exp_gyr_x;
                TaskControl_Monitor.GyrYCtl_PIDObj.exp = TaskControl_Monitor.exp_gyr_y;
            }

            TaskControl_Monitor.GyrZCtl_PIDObj.exp = TaskControl_Monitor.exp_gyr_z;
            TaskControl_AngularSpeedRing_PID_Update(&TaskControl_Monitor);

            /* test code */
            if (arm_state == DRONE_DISARM)
            {
                uint16_t moto_min = 0;
                uint16_t moto_idle = 0;
                uint16_t moto_max = 0;
                uint16_t moto_val = 0;

                for (uint8_t i = 0; i < 4; i++)
                {
                    SrvActuator.get_moto_control_range(i, &moto_min, &moto_idle, &moto_max);
                    moto_val = (uint16_t)((float)(exp_ctl_val->throttle_percent / 100.0f) * (moto_max - moto_min)) + moto_idle;
                    SrvActuator.moto_direct_drive(i, moto_val);
                }
            }
            /* test code */

            /* bug */
            // TaskControl_Actuator_ControlValue_Update(&TaskControl_Monitor);

            if(imu_err_code == SrvIMU_Sample_NoError)
            {
                for(axis = Axis_X; axis < Axis_Sum; axis ++)
                {
                    TaskControl_Monitor.acc_lst[axis] = TaskControl_Monitor.acc[axis];
                    TaskControl_Monitor.gyr_lst[axis] = TaskControl_Monitor.gyr[axis];
                }
            }
        
            return;
        }
    }

lock_moto:
    SrvActuator.lock();
}

static TaskControl_FlightParam_TypeDef TaskControl_Get_InuseParam(void)
{
    TaskControl_FlightParam_TypeDef Param;

    memset(&Param, 0, sizeof(TaskControl_FlightParam_TypeDef));

    Param.Outer.Pitch_Para.gP = TaskControl_Monitor.PitchCtl_PIDObj.gP;
    Param.Outer.Pitch_Para.gP_Diff_Max = TaskControl_Monitor.PitchCtl_PIDObj.diff_max;
    Param.Outer.Pitch_Para.gP_Diff_Min = TaskControl_Monitor.PitchCtl_PIDObj.diff_min;
    Param.Outer.Pitch_Para.gI = TaskControl_Monitor.PitchCtl_PIDObj.gI;
    Param.Outer.Pitch_Para.gI_Max = TaskControl_Monitor.PitchCtl_PIDObj.gI_Max;
    Param.Outer.Pitch_Para.gI_Min = TaskControl_Monitor.PitchCtl_PIDObj.gI_Min;
    Param.Outer.Pitch_Para.gD = TaskControl_Monitor.PitchCtl_PIDObj.gD;
    
    Param.Outer.Roll_Para.gP = TaskControl_Monitor.RollCtl_PIDObj.gP;
    Param.Outer.Roll_Para.gP_Diff_Max = TaskControl_Monitor.RollCtl_PIDObj.diff_max;
    Param.Outer.Roll_Para.gP_Diff_Min = TaskControl_Monitor.RollCtl_PIDObj.diff_min;
    Param.Outer.Roll_Para.gI = TaskControl_Monitor.RollCtl_PIDObj.gI;
    Param.Outer.Roll_Para.gI_Max = TaskControl_Monitor.RollCtl_PIDObj.gI_Max;
    Param.Outer.Roll_Para.gI_Min = TaskControl_Monitor.RollCtl_PIDObj.gI_Min;
    Param.Outer.Roll_Para.gD = TaskControl_Monitor.RollCtl_PIDObj.gD;

    Param.Inner.GyroX_Para.gP = TaskControl_Monitor.GyrXCtl_PIDObj.gP;
    Param.Inner.GyroX_Para.gP_Diff_Max = TaskControl_Monitor.GyrXCtl_PIDObj.diff_max;
    Param.Inner.GyroX_Para.gP_Diff_Min = TaskControl_Monitor.GyrXCtl_PIDObj.diff_min;
    Param.Inner.GyroX_Para.gI = TaskControl_Monitor.GyrXCtl_PIDObj.gI;
    Param.Inner.GyroX_Para.gI_Max = TaskControl_Monitor.GyrXCtl_PIDObj.gI_Max;
    Param.Inner.GyroX_Para.gI_Min = TaskControl_Monitor.GyrXCtl_PIDObj.gI_Min;
    Param.Inner.GyroX_Para.gD = TaskControl_Monitor.GyrXCtl_PIDObj.gD;
    
    Param.Inner.GyroY_Para.gP = TaskControl_Monitor.GyrYCtl_PIDObj.gP;
    Param.Inner.GyroY_Para.gP_Diff_Max = TaskControl_Monitor.GyrYCtl_PIDObj.diff_max;
    Param.Inner.GyroY_Para.gP_Diff_Min = TaskControl_Monitor.GyrYCtl_PIDObj.diff_min;
    Param.Inner.GyroY_Para.gI = TaskControl_Monitor.GyrYCtl_PIDObj.gI;
    Param.Inner.GyroY_Para.gI_Max = TaskControl_Monitor.GyrYCtl_PIDObj.gI_Max;
    Param.Inner.GyroY_Para.gI_Min = TaskControl_Monitor.GyrYCtl_PIDObj.gI_Min;
    Param.Inner.GyroY_Para.gD = TaskControl_Monitor.GyrYCtl_PIDObj.gD;

    Param.Inner.GyroZ_Para.gP = TaskControl_Monitor.GyrZCtl_PIDObj.gP;
    Param.Inner.GyroZ_Para.gP_Diff_Max = TaskControl_Monitor.GyrXCtl_PIDObj.diff_max;
    Param.Inner.GyroZ_Para.gP_Diff_Min = TaskControl_Monitor.GyrXCtl_PIDObj.diff_min;
    Param.Inner.GyroZ_Para.gI = TaskControl_Monitor.GyrZCtl_PIDObj.gI;
    Param.Inner.GyroZ_Para.gI_Max = TaskControl_Monitor.GyrZCtl_PIDObj.gI_Max;
    Param.Inner.GyroZ_Para.gI_Min = TaskControl_Monitor.GyrZCtl_PIDObj.gI_Min;
    Param.Inner.GyroZ_Para.gD = TaskControl_Monitor.GyrZCtl_PIDObj.gD;

    return Param;
}

/****************************************************** CLI Section ******************************************************************/
static void TaskControl_CLI_Polling(void)
{
    osEvent event;
    static TaskControl_CLIData_TypeDef CLIData;
    static uint16_t moto_ctl_buff[8] = {0};
    Shell *shell_obj = Shell_GetInstence();

    if(shell_obj == NULL)
        return;

    if(TaskControl_Monitor.CLIMessage_ID)
    {
        event = osMessageGet(TaskControl_Monitor.CLIMessage_ID, CLI_MESSAGE_OPEARATE_TIMEOUT);
        if (event.status == osEventMessage)
        {
            CLIData = *(TaskControl_CLIData_TypeDef *)(event.value.p);
            switch((uint8_t)CLIData.cli_type)
            {
                case TaskControl_Moto_Set_SpinDir:
                    memset(moto_ctl_buff, 0, sizeof(moto_ctl_buff));
                    SrvActuator.lock();
                    if(SrvActuator.reverse_spin(CLIData.index))
                    {
                        shellPrint(shell_obj, "moto spin dir set done\r\n");
                    }
                    else
                        shellPrint(shell_obj, "moto spin dir set error\r\n");
                    break;

                case TaskControl_Moto_Set_Spin:
                    if(CLIData.index < SrvActuator.get_cnt().moto_cnt)
                    {
                        for(uint8_t i = 0; i < SrvActuator.get_cnt().moto_cnt; i++)
                            moto_ctl_buff[i] = 0;

                        moto_ctl_buff[CLIData.index] = CLIData.value;
                    }
                    else
                    {
                        for(uint8_t i = 0; i < SrvActuator.get_cnt().moto_cnt; i++)
                            moto_ctl_buff[i] = CLIData.value;
                    }
                    break;

                case TaskControl_Moto_CliDisable:
                    memset(moto_ctl_buff, 0, sizeof(moto_ctl_buff));
                    TaskControl_Monitor.CLI_enable = false;
                    break;

                default: break;
            }

            SrvOsCommon.free(event.value.p);
        }
    }

    if(CLIData.cli_type == TaskControl_Moto_Set_Spin)
    {
        /* set moto spin */
        if (SrvActuator.moto_direct_drive)
        {
            for (uint8_t i = 0; i < SrvActuator.get_cnt().moto_cnt; i++)
                SrvActuator.moto_direct_drive(i, moto_ctl_buff[i]);
        }
    }
    else
        SrvActuator.lock();
}

static void TaskControl_CLI_MotoSpinTest(uint8_t moto_index, uint16_t test_val)
{
    uint8_t moto_num = SrvActuator.get_cnt().moto_cnt;
    bool arm_state = false;
    int16_t moto_max = 0;
    int16_t moto_idle = 0;
    int16_t moto_min = 0;
    uint32_t time_stamp = SrvOsCommon.get_os_ms();
    Shell *shell_obj = Shell_GetInstence();
    TaskControl_CLIData_TypeDef *p_CLIData = NULL;
    bool ctl_enable = true;

    if(shell_obj == NULL)
        return;

    SrvDataHub.get_arm_state(&arm_state);

    if(TaskControl_Monitor.CLIMessage_ID == NULL)
    {
        shellPrint(shell_obj, "TaskControl semaphore create failed\r\n");
        return;
    }

    if(arm_state == DRONE_DISARM)
    {
        TaskControl_Monitor.CLI_enable = false;
        shellPrint(shell_obj, "Set drone in ARM state first\r\n");
        return;
    }

    TaskControl_Monitor.CLI_enable = true;
    shellPrint(shell_obj, "make sure all propeller is already disassmabled\r\n");

    shellPrint(shell_obj, "moto count : %d\r\n", moto_num);
    if(moto_index >= moto_num)
    {
        shellPrint(shell_obj, "all moto selected\r\n");
        for (uint8_t i = 0; i < moto_num; i ++)
        {
            if (!SrvActuator.get_moto_control_range(i, &moto_min, &moto_idle, &moto_max))
            {
                ctl_enable = false;
                shellPrint(shell_obj, "moto %d control range down below\r\n", i);
                shellPrint(shell_obj, "\tmax  : %d\r\n", moto_max);
                shellPrint(shell_obj, "\tidle : %d\r\n", moto_idle);
                shellPrint(shell_obj, "\tmin  : %d\r\n", moto_min);
                break;
            }
        }
    }
    else
    {
        shellPrint(shell_obj, "moto %d is selected\r\n", moto_index);
        if(!SrvActuator.get_moto_control_range(moto_index, &moto_min, &moto_idle, &moto_max))
        {
            ctl_enable = false;
            shellPrint(shell_obj, "moto %d control range down below\r\n", moto_index);
            shellPrint(shell_obj, "\tmax  : %d\r\n", moto_max);
            shellPrint(shell_obj, "\tidle : %d\r\n", moto_idle);
            shellPrint(shell_obj, "\tmin  : %d\r\n", moto_min);
        }
    }

    if (!ctl_enable)
    {
        shellPrint(shell_obj, "Get moto control data range failed\r\n");
        return;
    }

    if(test_val > moto_max)
    {
        shellPrint(shell_obj, "input value [%d] is bigger than max [%d] value", test_val, moto_max);
    }
    else if(test_val < moto_min)
    {
        shellPrint(shell_obj, "input value [%d] is lower than min [%d] value", test_val, moto_min);
    }
    else
    {
        shellPrint(shell_obj, "current control value %d\r\n", test_val);

        p_CLIData = SrvOsCommon.malloc(sizeof(TaskControl_CLIData_TypeDef));
        if(p_CLIData)
        {
            p_CLIData->cli_type = TaskControl_Moto_Set_Spin;
            p_CLIData->index = moto_index;
            p_CLIData->timestamp = time_stamp;
            p_CLIData->value = test_val;

            if(osMessagePut(TaskControl_Monitor.CLIMessage_ID, (uint32_t)p_CLIData, CLI_MESSAGE_OPEARATE_TIMEOUT) != osOK)
            {
                SrvOsCommon.free(p_CLIData);
                shellPrint(shell_obj, "TaskControl CLI set failed\r\n");
            }
        }
        else
        {
            shellPrint(shell_obj, "Moto control test data malloc failed\r\n");
            SrvOsCommon.free(p_CLIData);
        }
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, moto_spin, TaskControl_CLI_MotoSpinTest, Single Moto Spin);

static void TaskControl_CLI_Set_MotoSpinDir(uint8_t moto_index, uint8_t dir)
{
    uint8_t moto_num = SrvActuator.get_cnt().moto_cnt;
    bool arm_state = false;
    uint32_t time_stamp = SrvOsCommon.get_os_ms();
    TaskControl_CLIData_TypeDef *p_CLIData = NULL;
    Shell *shell_obj = Shell_GetInstence();

    if(shell_obj == NULL)
        return;

    SrvDataHub.get_arm_state(&arm_state);
    if(TaskControl_Monitor.CLIMessage_ID == NULL)
    {
        shellPrint(shell_obj, "TaskControl semaphore create failed\r\n");
        return;
    }

    if(arm_state == DRONE_DISARM)
    {
        TaskControl_Monitor.CLI_enable = false;
        shellPrint(shell_obj, "Set drone in ARM state first\r\n");
        return;
    }

    TaskControl_Monitor.CLI_enable = true;
    shellPrint(shell_obj, "make sure all propeller is already disassmabled\r\n");
    if(moto_index >= moto_num)
    {
        shellPrint(shell_obj, "index over range\r\n");
        shellPrint(shell_obj, "arg must less than %d\r\n", (moto_num - 1));
        return;
    }
        
    p_CLIData = SrvOsCommon.malloc(sizeof(TaskControl_CLIData_TypeDef));
    if(p_CLIData == NULL)
    {
        shellPrint(shell_obj, "Moto direction setting data malloc failed\r\n");
        SrvOsCommon.free(p_CLIData);
        return;
    }
        
    shellPrint(shell_obj, "Setting time stamp %d\r\n", time_stamp);
    shellPrint(shell_obj, "0 ----> set spin clockwise \r\n");
    shellPrint(shell_obj, "1 ----> set spin anticlockwise \r\n");
            
    if(dir >= Actuator_Spin_AntiClockWise)
    {
        shellPrint(shell_obj, "current setting is anticlockwise\r\n");
        dir = Actuator_Spin_AntiClockWise;
    }
    else
        shellPrint(shell_obj, " current setting is clockwise\r\n");

    p_CLIData->cli_type = TaskControl_Moto_Set_SpinDir;
    p_CLIData->timestamp = time_stamp;
    p_CLIData->index = moto_index;
    p_CLIData->value = dir;

    if(osMessagePut(TaskControl_Monitor.CLIMessage_ID, (uint32_t)p_CLIData, CLI_MESSAGE_OPEARATE_TIMEOUT) != osOK)
    {
        shellPrint(shell_obj, "TaskControl CLI set failed\r\n");
        SrvOsCommon.free(p_CLIData);
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, Set_Moto_Dir, TaskControl_CLI_Set_MotoSpinDir, Set Moto Spin Direction);

static void TaskControl_Close_CLI(void)
{
    Shell *shell_obj = Shell_GetInstence();
    TaskControl_CLIData_TypeDef *p_CLIData = NULL;
    uint32_t time_stamp = SrvOsCommon.get_os_ms();

    if(shell_obj == NULL)
        return;

    if(TaskControl_Monitor.CLIMessage_ID)
    {
        shellPrint(shell_obj, "TaskControl semaphore create failed\r\n");
        return;
    }

    shellPrint(shell_obj, "TaskControl CLI disable\r\n");
    p_CLIData = SrvOsCommon.malloc(sizeof(TaskControl_CLIData_TypeDef));
    if (p_CLIData == NULL)
    {
        shellPrint(shell_obj, "Moto control test data malloc failed\r\n");
        SrvOsCommon.free(p_CLIData);
        return;
    }

    p_CLIData->cli_type = TaskControl_Moto_CliDisable;
    p_CLIData->timestamp = time_stamp;
    p_CLIData->index = 0;
    p_CLIData->value = 0;

    if(osMessagePut(TaskControl_Monitor.CLIMessage_ID, (uint32_t)p_CLIData, CLI_MESSAGE_OPEARATE_TIMEOUT) != osOK)
    {
        shellPrint(shell_obj, "TaskControl CLI set failed\r\n");
        SrvOsCommon.free(p_CLIData);
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, disable_actuator_test, TaskControl_Close_CLI, disable actuator test);

static bool TaskControl_PID_Param_Print(Shell *obj, PIDObj_TypeDef para)
{
    if (obj)
    {
        shellPrint(obj, "\t gP:          %f \r\n", para.gP);
        shellPrint(obj, "\t P-Diff Max:  %f \r\n", para.diff_max);
        shellPrint(obj, "\t P-Diff Min:  %f \r\n", para.diff_min);
        shellPrint(obj, "\t gI:          %f \r\n", para.gI);
        shellPrint(obj, "\t I-Max:       %f \r\n", para.gI_Max);
        shellPrint(obj, "\t I-Min:       %f \r\n", para.gI_Min);
        shellPrint(obj, "\t gD:          %f \r\n", para.gD);
        shellPrint(obj, "\r\n");

        return true;
    }

    return false;
}

static void TaskControl_Get_Inner_Controller_Parameter(void)
{
    Shell *shell_obj = Shell_GetInstence();

    if(shell_obj == NULL)
        return;
    
    shellPrint(shell_obj, "GyroX PID\r\n");
    TaskControl_PID_Param_Print(shell_obj, TaskControl_Monitor.GyrXCtl_PIDObj);
    
    shellPrint(shell_obj, "GyroY PID\r\n");
    TaskControl_PID_Param_Print(shell_obj, TaskControl_Monitor.GyrYCtl_PIDObj);

    shellPrint(shell_obj, "GyroZ PID\r\n");
    TaskControl_PID_Param_Print(shell_obj, TaskControl_Monitor.GyrZCtl_PIDObj);
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, PID_Inner_Param, TaskControl_Get_Inner_Controller_Parameter, get controller parameter);

static void TaskControl_Get_Outer_Controller_Parameter(void)
{
    Shell *shell_obj = Shell_GetInstence();

    if(shell_obj == NULL)
        return;
        
    shellPrint(shell_obj, "Pitch PID\r\n");
    TaskControl_PID_Param_Print(shell_obj, TaskControl_Monitor.PitchCtl_PIDObj);

    shellPrint(shell_obj, "Roll PID\r\n");
    TaskControl_PID_Param_Print(shell_obj, TaskControl_Monitor.RollCtl_PIDObj);
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, PID_Outer_Param, TaskControl_Get_Outer_Controller_Parameter, get controller parameter);

static void TaskControl_Param_Set(const char *sel, const char *para_sel, uint16_t value, const char *save)
{
    Shell *shell_obj = Shell_GetInstence();
    PIDObj_TypeDef *PID_Obj = NULL;
    TaskControl_FlightParam_TypeDef Param;

    if(shell_obj == NULL)
        return;

    memset(&Param, 0, sizeof(TaskControl_FlightParam_TypeDef));

    shellPrint(shell_obj, "\tinput param 1 selection list\r\n");
    shellPrint(shell_obj, "\t pitch ----- set pitch  pid object \r\n");
    shellPrint(shell_obj, "\t roll  ----- set roll   pid object \r\n");
    shellPrint(shell_obj, "\t gx -------- set gyro x pid object \r\n");
    shellPrint(shell_obj, "\t gy -------- set gyro y pid object \r\n");
    shellPrint(shell_obj, "\t gz -------- set gyro z pid object \r\n");

    shellPrint(shell_obj, "\tinput param 2 PID param select\r\n");
    shellPrint(shell_obj, "\t gP -------- set PID gP section \r\n");
    shellPrint(shell_obj, "\t gP_Diff --- set PID gP_Diff section \r\n");
    shellPrint(shell_obj, "\t gI -------- set PID gI section \r\n");
    shellPrint(shell_obj, "\t gI_range -- set PID gI Range section \r\n");
    shellPrint(shell_obj, "\t gD -------- set PID gD section \r\n");

    shellPrint(shell_obj, "\tinput param 3 incomming value\r\n");
    shellPrint(shell_obj, "\t for example: if u want set 0.81 then input 810 \r\n");
    shellPrint(shell_obj, "\t              actually value = set value / 1000.0f \r\n");
    shellPrint(shell_obj, "\tinput param 4 save choice\r\n");
    shellPrint(shell_obj, "\t S ------ save new parameter \r\n");
    
    shellPrint(shell_obj, "\r\n\t Input value : %d \r\n", value / 1000.0f);
    shellPrint(shell_obj, "\r\n");

    if ((strlen(sel) == strlen("pitch")) && (memcmp(sel, "pitch", strlen("pitch")) == 0))
    {
        shellPrint(shell_obj, "\t[ pitch selected ]\r\n");
        PID_Obj = &TaskControl_Monitor.PitchCtl_PIDObj;
    }
    else if ((strlen(sel) == strlen("roll")) && (memcmp(sel, "roll", strlen("roll")) == 0))
    {
        shellPrint(shell_obj, "\t[ roll selected ]\r\n");
        PID_Obj = &TaskControl_Monitor.RollCtl_PIDObj;
    }
    else if ((strlen(sel) == strlen("gx")) && (memcmp(sel, "gx", strlen("gx")) == 0))
    {
        shellPrint(shell_obj, "\t[ gyro x selected ]\r\n");
        PID_Obj = &TaskControl_Monitor.GyrXCtl_PIDObj;
    }
    else if ((strlen(sel) == strlen("gy")) && (memcmp(sel, "gy", strlen("gy")) == 0))
    {
        shellPrint(shell_obj, "\t[ gyro y selected ]\r\n");
        PID_Obj = &TaskControl_Monitor.GyrYCtl_PIDObj;
    }
    else if ((strlen(sel) == strlen("gz")) && (memcmp(sel, "gz", strlen("gz")) == 0))
    {
        shellPrint(shell_obj, "\t[ gyro z selected ]\r\n");
        PID_Obj = &TaskControl_Monitor.GyrZCtl_PIDObj;
    }
    else
    {
        shellPrint(shell_obj, "\tUnkonw Selection\r\n");
        return;
    }

    if ((strlen(para_sel) == strlen("gP")) && \
        (memcmp(para_sel, "gP", strlen("gP")) == 0))
    {
        shellPrint(shell_obj, "\t[ gP selected ]\r\n");
        PID_Obj->gP = value / 1000.0f;
    }
    else if ((strlen(para_sel) == strlen("gP_Diff")) && \
             (memcmp(para_sel, "gP_Diff", strlen("gP_Diff")) == 0))
    {
        shellPrint(shell_obj, "\t[ gP_Diff selected ]\r\n");
        PID_Obj->diff_max = value / 1000.0f;
        PID_Obj->diff_min = -value / 1000.0f;
    }
    else if ((strlen(para_sel) == strlen("gI")) && \
             (memcmp(para_sel, "gI", strlen("gI")) == 0))
    {
        shellPrint(shell_obj, "\t[ gI selected ]\r\n");
        PID_Obj->gI = value / 1000.0f;
    }
    else if ((strlen(para_sel) == strlen("gI_Range")) && \
             (memcmp(para_sel, "gI_Range", strlen("gI_Range")) == 0))
    {
        shellPrint(shell_obj, "\t[ gI_Range selected ]\r\n");
        PID_Obj->gI_Max = value / 1000.0f;
        PID_Obj->gI_Min = -value / 1000.0f;
    }
    else if ((strlen(para_sel) == strlen("gD")) && \
             (memcmp(para_sel, "gD", strlen("gD")) == 0))
    {
        shellPrint(shell_obj, "\t[ gD selected ]\r\n");
        PID_Obj->gD = value / 1000.0f;
    }

    /* show new param */
    shellPrint(shell_obj, "[ - New Param - ]\r\n");
    TaskControl_PID_Param_Print(shell_obj, *PID_Obj);

    if (save && (strlen(save) == 1) && (*save == 'S'))
    {
        if (TaskControl_Monitor.pid_store_info.item_addr && \
            TaskControl_Monitor.pid_store_info.item.data_addr)
        {
            Param = TaskControl_Get_InuseParam();
            if (Storage.update(Para_User, TaskControl_Monitor.pid_store_info.item.data_addr, \
                               (uint8_t *)&Param, sizeof(TaskControl_FlightParam_TypeDef)) == Storage_Error_None)
            {
                shellPrint(shell_obj, "\t[ -- PID Parameter Store Accomplish -- ]\r\n");
                return;
            }

            shellPrint(shell_obj, "\t[ -- PID Parameter Store Failed -- ]\r\n");
        }
        else
        {
            shellPrint(shell_obj, "\t[ item slot addr : %lld ]\r\n", TaskControl_Monitor.pid_store_info.item_addr);
            shellPrint(shell_obj, "\t[ data slot addr : %lld ]\r\n", TaskControl_Monitor.pid_store_info.item.data_addr);
            shellPrint(shell_obj, "\t[ -- PID parameter Storage Error -- ]\r\n");
        }
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, Set_PID, TaskControl_Param_Set, set PID gP Paramter);

