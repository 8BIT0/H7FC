/*
 *  coder : 8_B!T0
 *  this file use for moto & servo control
 *  pitch & roll control mode outer loop is angular loop inner loop is angular speed loop 
 *  yaw control mode is only angular speed loop
 */
#include "Task_Control.h"
#include "Srv_OsCommon.h"
#include "Srv_CtlDataArbitrate.h"
#include "Task_Telemetry.h"
#include "Srv_DataHub.h"
#include "Srv_Actuator.h"
#include "shell_port.h"

#define ACTUATOR_STORAGE_SCTION_NAME "Actuator_Setting"
#define CONTROL_STORAGE_SECTION_NAME "PID_Para"

#define DEFAULT_CONTROL_MODEL Model_Quad
#define DEFAULT_ESC_TYPE DevDshot_600

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

DataPipe_CreateDataObj(ControlData_TypeDef, Smp_Inuse_CtlData);

static uint32_t imu_update_time = 0;
static uint32_t att_update_time = 0;
static uint32_t tunning_time_stamp = 0;
static uint8_t imu_err_code;
static bool arm_state = true;
static bool failsafe = false;
static bool imu_init_state = false;
static bool att_update = false;
static bool tunning_state = false;
static bool configrator_attach = false;
static bool control_enable = false;

SrvIMU_UnionData_TypeDef LstCyc_IMU_Data;
SrvRecever_RCSig_TypeDef LstCyc_Rc_Data;

TaskControl_Monitor_TypeDef TaskControl_Monitor = {
    .init_state = false,
    .control_abort = false,

    /* on test mode use angular_speed over rate threshold protect */
    .angular_protect_enable = true,
    .angular_protect = false,

    /* on test mode for throttle control value mutation protect */
    .throttle_protect_enable = true,
    .throttle_percent = false,

    .actuator_model = Model_Quad,
    .IMU_Rt = 0,
};

/* internal function */
static bool TaskControl_AttitudeRing_PID_Update(TaskControl_Monitor_TypeDef *monitor, bool att_state);
static bool TaskControl_AngularSpeedRing_PID_Update(TaskControl_Monitor_TypeDef *monitor);
static void TaskControl_FlightControl_Polling(Srv_CtlExpectionData_TypeDef *exp_ctl_val);
static void TaskControl_Actuator_ControlValue_Update(TaskControl_Monitor_TypeDef *monitor);
static void TaskControl_CLI_Polling(void);
static bool TaskControl_Get_StoreParam(void);
static TaskControl_FlightParam_TypeDef TaskControl_Get_InuseParam(void);

/* internal var */
static uint32_t TaskControl_Period = 0;

void TaskControl_Init(uint32_t period)
{
    uint8_t i = 0;
    Srv_CtlRange_TypeDef att_ctl_range[Att_Ctl_Sum];
    Srv_CtlRange_TypeDef angularspeed_ctl_range[Axis_Sum];
    Storage_ItemSearchOut_TypeDef search_out;

    // init monitor
    memset(&TaskControl_Monitor, 0, sizeof(TaskControl_Monitor));
    memset(&att_ctl_range, 0, sizeof(Srv_CtlRange_TypeDef));
    memset(&angularspeed_ctl_range, 0, sizeof(Srv_CtlRange_TypeDef));

    /* read actuator setting in storage */
    search_out = Storage.search(External_Flash, Para_User, ACTUATOR_STORAGE_SCTION_NAME);
    if (search_out.item_addr)
    {
        /* parameter matched */
        TaskControl_Monitor.actuator_store_item = search_out.item;
        if (Storage.get(External_Flash, Para_User, search_out.item, &TaskControl_Monitor.actuator_setting, sizeof(SrvActuatot_Setting_TypeDef)) != Storage_Error_None)
        {
            /* use defaule actuator setting */
        }
    }

    TaskControl_Monitor.init_state = SrvActuator.init(DEFAULT_CONTROL_MODEL, DEFAULT_ESC_TYPE);
    TaskControl_Monitor.actuator_model = SrvActuator.get_model();

    /* PID Parametet Init */
    TaskControl_Get_StoreParam();

    osMessageQDef(MotoCLI_Data, 64, TaskControl_CLIData_TypeDef);
    TaskControl_Monitor.CLIMessage_ID = osMessageCreate(osMessageQ(MotoCLI_Data), NULL);
    
    memset(DataPipe_DataObjAddr(Smp_Inuse_CtlData), 0, DataPipe_DataSize(Smp_Inuse_CtlData));
    InUseCtlData_Smp_DataPipe.data_addr = DataPipe_DataObjAddr(Smp_Inuse_CtlData);
    InUseCtlData_Smp_DataPipe.data_size = DataPipe_DataSize(Smp_Inuse_CtlData);
    DataPipe_Enable(&InUseCtlData_Smp_DataPipe);

    /* set control range */
    /* attitude control range */
    for(i = Att_Pitch; i < Att_Ctl_Sum; i++)
    {
        att_ctl_range[i].max = 25.0f;   /* max attitude control angle ±50 deg */
        att_ctl_range[i].min = -25.0f;
        att_ctl_range[i].idle = 0.0f;
        att_ctl_range[i].dead_zone_max = 0.5f;
        att_ctl_range[i].dead_zone_min = -0.5f;
        att_ctl_range[i].enable_dead_zone = true;
    }

    /* X&Y axis angular speed control range */
    for(i = Axis_X; i < Axis_Z; i++)
    {
        angularspeed_ctl_range[i].max = 300.0f;
        angularspeed_ctl_range[i].min = -300.0f;
        angularspeed_ctl_range[i].idle = 0.0f;
        angularspeed_ctl_range[i].enable_dead_zone = false;
    }
        
    angularspeed_ctl_range[Axis_Z].max = 100.0f;
    angularspeed_ctl_range[Axis_Z].min = -100.0f;
    angularspeed_ctl_range[Axis_Z].idle = 0.0f;
    angularspeed_ctl_range[Axis_Z].enable_dead_zone = false;
    
    control_enable = Srv_CtlDataArbitrate.init(att_ctl_range, angularspeed_ctl_range);
    TaskControl_Monitor.moto_cnt = SrvActuator.get_cnt().moto_cnt;
    TaskControl_Monitor.servo_cnt = SrvActuator.get_cnt().servo_cnt;

    if(TaskControl_Monitor.moto_cnt)
    {
        TaskControl_Monitor.moto_value = SrvOsCommon.malloc(sizeof(uint16_t) * TaskControl_Monitor.moto_cnt);
    
        if(TaskControl_Monitor.moto_value == NULL)
            SrvOsCommon.free(TaskControl_Monitor.moto_value);
    }

    if(TaskControl_Monitor.servo_cnt)
    {
        TaskControl_Monitor.servo_value = SrvOsCommon.malloc(sizeof(uint16_t) * TaskControl_Monitor.servo_cnt);
    
        if(TaskControl_Monitor.servo_value == NULL)
            SrvOsCommon.free(TaskControl_Monitor.servo_value);
    }

    TaskControl_Period = period;
}

static TaskControl_FlightParam_TypeDef TaskControl_Get_DefaultParam(void)
{
    TaskControl_FlightParam_TypeDef Param;

    memset(&Param, 0, sizeof(TaskControl_FlightParam_TypeDef));

    /* use default pid data */
    /* attitude PID control parameter section */
    Param.Outer.Pitch_Para.gP_Diff_Max = ATTITUDE_PID_DIFF_MAX;
    Param.Outer.Roll_Para.gP_Diff_Max = ATTITUDE_PID_DIFF_MAX;

    Param.Outer.Pitch_Para.gP_Diff_Min = ATTITUDE_PID_DIFF_MIN;
    Param.Outer.Roll_Para.gP_Diff_Min = ATTITUDE_PID_DIFF_MIN;

    Param.Outer.Pitch_Para.gP = 1.2;
    Param.Outer.Pitch_Para.gI = 0.08;
    Param.Outer.Pitch_Para.gI_Max = 50;
    Param.Outer.Pitch_Para.gI_Min = -50;
    Param.Outer.Pitch_Para.gD = 1;

    Param.Outer.Roll_Para.gP = 1.2;
    Param.Outer.Roll_Para.gI = 0.08;
    Param.Outer.Roll_Para.gI_Max = 50;
    Param.Outer.Roll_Para.gI_Min = -50;
    Param.Outer.Roll_Para.gD = 1;

    /* angular PID control parameter section */
    Param.Inner.GyroX_Para.gP_Diff_Max = GYRO_X_RATE_PID_DIFF_MAX;
    Param.Inner.GyroX_Para.gP_Diff_Min = GYRO_X_RATE_PID_DIFF_MIN;

    Param.Inner.GyroX_Para.gP = 2;
    Param.Inner.GyroX_Para.gI = 0.002;
    Param.Inner.GyroX_Para.gI_Max = 30;
    Param.Inner.GyroX_Para.gI_Min = -30;
    Param.Inner.GyroX_Para.gD = 0.1;

    Param.Inner.GyroY_Para.gP_Diff_Max = GYRO_Y_RATE_PID_DIFF_MAX;
    Param.Inner.GyroY_Para.gP_Diff_Min = GYRO_Y_RATE_PID_DIFF_MIN;
    
    Param.Inner.GyroY_Para.gP = 2;
    Param.Inner.GyroY_Para.gI = 0.002;
    Param.Inner.GyroY_Para.gI_Max = 30;
    Param.Inner.GyroY_Para.gI_Min = -30;
    Param.Inner.GyroY_Para.gD = 0.1;

    Param.Inner.GyroZ_Para.gP_Diff_Max = GYRO_X_RATE_PID_DIFF_MAX;
    Param.Inner.GyroZ_Para.gP_Diff_Min = GYRO_X_RATE_PID_DIFF_MIN;
    
    Param.Inner.GyroZ_Para.gP = 1;
    Param.Inner.GyroZ_Para.gI = 0.02;
    Param.Inner.GyroZ_Para.gI_Max = 30;
    Param.Inner.GyroZ_Para.gI_Min = -30;
    Param.Inner.GyroZ_Para.gD = 0.1;

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
static bool TaskControl_Get_StoreParam(void)
{
    TaskControl_FlightParam_TypeDef Param;
    TaskControl_FlightParam_TypeDef Default_Param;
    TaskControl_FlightParam_TypeDef *p_UseParam = NULL;
    bool state = true;

    /* search storage section first */
    memset(&Param, 0, sizeof(TaskControl_FlightParam_TypeDef));
    memset(&Default_Param, 0, sizeof(TaskControl_FlightParam_TypeDef));
    memset(&TaskControl_Monitor.param_match_state, 0, sizeof(Storage_ItemSearchOut_TypeDef));
    TaskControl_Monitor.param_match_state = Storage.search(External_Flash, Para_User, CONTROL_STORAGE_SECTION_NAME);

    Param = TaskControl_Get_DefaultParam();
    Default_Param = Param;
    p_UseParam = &Default_Param;
    
    if (TaskControl_Monitor.param_match_state.item_addr == 0)
    {
        /* no pid parameter found in external flash chip under user partten */
        /* section create successful */
        if (Storage.create(External_Flash, Para_User, CONTROL_STORAGE_SECTION_NAME, (uint8_t *)&Param, sizeof(TaskControl_FlightParam_TypeDef)) != Storage_Error_None)
            state = false;
    }
    else
    {
        if ((TaskControl_Monitor.param_match_state.item.len == sizeof(TaskControl_FlightParam_TypeDef)) && \
            (Storage.get(External_Flash, Para_User, TaskControl_Monitor.param_match_state.item, (uint8_t *)&Param, sizeof(TaskControl_FlightParam_TypeDef)) == Storage_Error_None))
        {
            p_UseParam = &Param;
        }
        else
            state = false;
    }

    TaskControl_Param_Copy(&TaskControl_Monitor.PitchCtl_PIDObj, p_UseParam->Outer.Pitch_Para);
    TaskControl_Param_Copy(&TaskControl_Monitor.RollCtl_PIDObj,  p_UseParam->Outer.Roll_Para);
    TaskControl_Param_Copy(&TaskControl_Monitor.GyrXCtl_PIDObj,  p_UseParam->Inner.GyroX_Para);
    TaskControl_Param_Copy(&TaskControl_Monitor.GyrYCtl_PIDObj,  p_UseParam->Inner.GyroY_Para);
    TaskControl_Param_Copy(&TaskControl_Monitor.GyrZCtl_PIDObj,  p_UseParam->Inner.GyroZ_Para);

    return state;
}

void TaskControl_Core(void const *arg)
{
    uint32_t sys_time = SrvOsCommon.get_os_ms();
    ControlData_TypeDef CtlData;
    Srv_CtlExpectionData_TypeDef Cnv_CtlData;
    Srv_CtlExpectionData_TypeDef Lst_CtlData;
    
    memset(&CtlData, 0, sizeof(ControlData_TypeDef));
    memset(&Cnv_CtlData, 0, sizeof(Srv_CtlExpectionData_TypeDef));
    memset(&Lst_CtlData, 0, sizeof(Srv_CtlExpectionData_TypeDef));

    while(1)
    {
        Srv_CtlDataArbitrate.negociate_update(&CtlData);
        
        if(control_enable && !TaskControl_Monitor.CLI_enable)
        {
            Cnv_CtlData = Srv_CtlDataArbitrate.get_data();
            TaskControl_FlightControl_Polling(&Cnv_CtlData);
            
            CtlData.exp_gyr_x = Cnv_CtlData.exp_angularspeed[Axis_X];
            CtlData.exp_gyr_y = Cnv_CtlData.exp_angularspeed[Axis_Y];
            CtlData.exp_gyr_z = Cnv_CtlData.exp_angularspeed[Axis_Z];

            Lst_CtlData = Cnv_CtlData;
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

        DataPipe_DataObj(Smp_Inuse_CtlData) = CtlData;
        /* pipe in use control data to data hub */
        DataPipe_SendTo(&InUseCtlData_Smp_DataPipe, &InUseCtlData_hub_DataPipe);

        SrvOsCommon.precise_delay(&sys_time, TaskControl_Period);
    }
}

static bool TaskControl_AttitudeRing_PID_Update(TaskControl_Monitor_TypeDef *monitor, bool att_state)
{
    if(monitor)
    {
        monitor->att_pid_state = att_state;
        
        if(att_state)
        {
           /* pitch PID update */
            PID_Update(&monitor->PitchCtl_PIDObj, monitor->attitude.pitch, monitor->exp_attitude.pitch);

            /* roll PID update */
            PID_Update(&monitor->RollCtl_PIDObj, monitor->attitude.roll, monitor->exp_attitude.roll);

            return true;
        }
    }

    return false;
}

static bool TaskControl_AngularSpeedRing_PID_Update(TaskControl_Monitor_TypeDef *monitor)
{
    if(monitor && monitor->att_pid_state)
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
        ctl_buf[Actuator_Ctl_Throttle] = monitor->throttle_percent;

        ctl_buf[Actuator_Ctl_GyrX] = monitor->GyrXCtl_PIDObj.fout;
        ctl_buf[Actuator_Ctl_GyrY] = monitor->GyrYCtl_PIDObj.fout;
        ctl_buf[Actuator_Ctl_GyrZ] = monitor->GyrZCtl_PIDObj.fout;
    }

    SrvActuator.moto_control(ctl_buf);
}

/****************************************************** Flight Control Section ********************************************************/
/* need to be optmize */
static void TaskControl_FlightControl_Polling(Srv_CtlExpectionData_TypeDef *exp_ctl_val)
{
    uint8_t axis = Axis_X;
    uint32_t tunning_port = 0;

    if (TaskControl_Monitor.init_state && !TaskControl_Monitor.control_abort)
    {
        imu_init_state = false;

        // get failsafe
        SrvDataHub.get_arm_state(&arm_state);
        SrvDataHub.get_failsafe(&failsafe);
        
        SrvDataHub.get_tunning_state(&tunning_time_stamp, &tunning_state, &tunning_port);

        /* if in tunning or attach configrator then lock moto */
        if(tunning_state || configrator_attach)
        {
            goto lock_moto;
        }

        // get imu init state first
        if(!SrvDataHub.get_imu_init_state(&imu_init_state) || !imu_init_state)
        {
            /* imu init error then lock the acturator */
            failsafe = true;
            arm_state = TELEMETRY_SET_ARM;
                
            goto lock_moto;
        }

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
                                      &imu_err_code))
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
                    TaskControl_Monitor.control_abort = true;
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
                        {
                            TaskControl_Monitor.angular_protect = true;
                        }
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

        if(exp_ctl_val->recover_flip_over && TaskControl_Monitor.flip_over)
        {
            /* when drone is up side down and we want to flip over it by telemetry */
            /* reverse propeller spin dir to reverse the drone */
        }
        else
        {
             /* do drone control algorithm down below */
             TaskControl_Monitor.throttle_percent = exp_ctl_val->throttle_percent;

            /* Update PID */
            if(exp_ctl_val->mode == Attitude_Control)
            {
                /* set expection attitude */
                TaskControl_Monitor.RollCtl_PIDObj.exp = exp_ctl_val->exp_attitude[Att_Roll];
                TaskControl_Monitor.PitchCtl_PIDObj.exp = exp_ctl_val->exp_attitude[Att_Pitch];
    
                TaskControl_AttitudeRing_PID_Update(&TaskControl_Monitor, att_update);
                
                TaskControl_Monitor.GyrXCtl_PIDObj.exp = TaskControl_Monitor.RollCtl_PIDObj.fout;
                TaskControl_Monitor.GyrYCtl_PIDObj.exp = TaskControl_Monitor.PitchCtl_PIDObj.fout;

                exp_ctl_val->exp_angularspeed[Axis_X] = TaskControl_Monitor.RollCtl_PIDObj.fout;
                exp_ctl_val->exp_angularspeed[Axis_Y] = TaskControl_Monitor.PitchCtl_PIDObj.fout;
            }
            else
            {
                TaskControl_Monitor.GyrXCtl_PIDObj.exp = exp_ctl_val->exp_angularspeed[Axis_X];
                TaskControl_Monitor.GyrYCtl_PIDObj.exp = exp_ctl_val->exp_angularspeed[Axis_Y];
            }

            TaskControl_Monitor.GyrZCtl_PIDObj.exp = exp_ctl_val->exp_angularspeed[Axis_Z];
            TaskControl_AngularSpeedRing_PID_Update(&TaskControl_Monitor);
            TaskControl_Actuator_ControlValue_Update(&TaskControl_Monitor);

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

        switch((uint8_t)event.status)
        {
            case osEventMessage:
                CLIData = *(TaskControl_CLIData_TypeDef *)(event.value.p);

                switch((uint8_t)CLIData.cli_type)
                {
                    case TaskControl_Moto_Set_SpinDir:
                        memset(moto_ctl_buff, 0, sizeof(moto_ctl_buff));
                        if(SrvActuator.invert_spin(CLIData.index))
                        {
                            shellPrint(shell_obj, "moto spin dir set done\r\n");
                        }
                        else
                            shellPrint(shell_obj, "moto spin dir set error\r\n");
                        break;

                    case TaskControl_Moto_Set_Spin:
                        if(CLIData.index < SrvActuator.get_cnt().moto_cnt)
                        {
                            moto_ctl_buff[CLIData.index] = CLIData.value;
                        }
                        else
                        {
                            for(uint8_t i = 0; i < SrvActuator.get_cnt().moto_cnt; i++)
                            {
                                moto_ctl_buff[i] = CLIData.value;
                            }
                        }
                        break;

                    case TaskControl_Moto_CliDisable:
                        memset(moto_ctl_buff, 0, sizeof(moto_ctl_buff));
                        TaskControl_Monitor.CLI_enable = false;
                        break;

                    default:
                        break;
                }

                SrvOsCommon.free(event.value.p);
                break;

            case osEventSignal:
            case osEventMail:
            case osEventTimeout:
            case osErrorParameter:
            case osErrorResource:
            case osErrorTimeoutResource:
            case osErrorISR:
            case osErrorISRRecursive:
            case osErrorPriority:
            case osErrorNoMemory:
            case osErrorValue:
            case osErrorOS:
            case os_status_reserved:
            default:
                break;
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

static void TaskControl_CLI_AllMotoSpinTest(uint16_t test_val)
{
    bool arm_state = false;
    Shell *shell_obj = Shell_GetInstence();

    if(shell_obj == NULL)
        return;

    SrvDataHub.get_arm_state(&arm_state);

    if(TaskControl_Monitor.CLIMessage_ID)
    {
        if(arm_state == DRONE_ARM)
        {
            TaskControl_Monitor.CLI_enable = true;
        }
        else
        {
            TaskControl_Monitor.CLI_enable = false;
            shellPrint(shell_obj, "Set drone in ARM state first\r\n");
        }
    }
    else
    {
        shellPrint(shell_obj, "TaskControl semaphore create failed\r\n");
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, All_Moto_Spin, TaskControl_CLI_AllMotoSpinTest, All Moto Spin);

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

    if(shell_obj == NULL)
        return;

    SrvDataHub.get_arm_state(&arm_state);

    if(TaskControl_Monitor.CLIMessage_ID)
    {
        if(arm_state == DRONE_ARM)
        {
            TaskControl_Monitor.CLI_enable = true;
            shellPrint(shell_obj, "make sure all propeller is already disassmabled\r\n");

            shellPrint(shell_obj, "moto count : %d\r\n", moto_num);
            if(moto_index >= moto_num)
            {
                shellPrint(shell_obj, "index over range\r\n");
                shellPrint(shell_obj, "arg must less than %d\r\n", (moto_num - 1));
            }
            else
            {
                if(SrvActuator.get_moto_control_range(moto_index, &moto_min, &moto_idle, &moto_max))
                {
                    shellPrint(shell_obj, "moto %d is selected\r\n", moto_index);
                    shellPrint(shell_obj, "moto max  : %d\r\n", moto_max);
                    shellPrint(shell_obj, "moto idle : %d\r\n", moto_idle);
                    shellPrint(shell_obj, "moto min  : %d\r\n", moto_min);

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
                else
                {
                    shellPrint(shell_obj, "Get moto control data range failed\r\n");
                }
            }
        }
        else
        {
            TaskControl_Monitor.CLI_enable = false;
            shellPrint(shell_obj, "Set drone in ARM state first\r\n");
        }
    }
    else
    {
        shellPrint(shell_obj, "TaskControl semaphore create failed\r\n");
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, Single_Moto_Spin, TaskControl_CLI_MotoSpinTest, Single Moto Spin);

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
    
    if(TaskControl_Monitor.CLIMessage_ID)
    {
        if(arm_state == DRONE_ARM)
        {
            TaskControl_Monitor.CLI_enable = true;

            shellPrint(shell_obj, "make sure all propeller is already disassmabled\r\n");

            if(moto_index >= moto_num)
            {
                shellPrint(shell_obj, "index over range\r\n");
                shellPrint(shell_obj, "arg must less than %d\r\n", (moto_num - 1));
            }
            else
            {
                p_CLIData = SrvOsCommon.malloc(sizeof(TaskControl_CLIData_TypeDef));

                if(p_CLIData == NULL)
                {
                    shellPrint(shell_obj, "Moto direction setting data malloc failed\r\n");
                    SrvOsCommon.free(p_CLIData);
                }
                else
                {
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
            }
        }
        else
        {
            TaskControl_Monitor.CLI_enable = false;
            shellPrint(shell_obj, "Set drone in ARM state first\r\n");
        }
    }
    else
    {
        shellPrint(shell_obj, "TaskControl semaphore create failed\r\n");
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
        shellPrint(shell_obj, "TaskControl CLI disable\r\n");
        p_CLIData = SrvOsCommon.malloc(sizeof(TaskControl_CLIData_TypeDef));
        if (p_CLIData)
        {
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
        else
        {
            shellPrint(shell_obj, "Moto control test data malloc failed\r\n");
            SrvOsCommon.free(p_CLIData);
        }
    }
    else
    {
        shellPrint(shell_obj, "TaskControl semaphore create failed\r\n");
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
        if (TaskControl_Monitor.param_match_state.item_addr && \
            TaskControl_Monitor.param_match_state.item.data_addr)
        {
            Param = TaskControl_Get_InuseParam();
            if (Storage.update( External_Flash, Para_User, \
                                TaskControl_Monitor.param_match_state.item.data_addr, \
                                (uint8_t *)&Param, sizeof(TaskControl_FlightParam_TypeDef)) == Storage_Error_None)
            {
                shellPrint(shell_obj, "\t[ -- PID Parameter Store Accomplish -- ]\r\n");
                return;
            }

            shellPrint(shell_obj, "\t[ -- PID Parameter Store Failed -- ]\r\n");
        }
        else
        {
            shellPrint(shell_obj, "\t[ item slot addr : %lld ]\r\n", TaskControl_Monitor.param_match_state.item_addr);
            shellPrint(shell_obj, "\t[ data slot addr : %lld ]\r\n", TaskControl_Monitor.param_match_state.item.data_addr);
            shellPrint(shell_obj, "\t[ -- PID parameter Storage Error -- ]\r\n");
        }
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, Set_PID, TaskControl_Param_Set, set PID gP Paramter);

