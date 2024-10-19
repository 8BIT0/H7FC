/*
 *  Auther : 8_B!T0
 *  this file use for moto & servo control
 *  (but currently none servo)
 */
#include "Task_Control.h"
#include "Srv_OsCommon.h"
#include "Task_Telemetry.h"
#include "Srv_DataHub.h"
#include "Srv_Actuator.h"
#include "shell_port.h"

#define CONTROL_STORAGE_SECTION_NAME "Control_Para"

#define DEFAULT_ATTITUDE_CONTROLLER_MODE CtlM_PID
#define DEFAULT_ALTITUDE_CONTROLLER_MODE CtlM_PID

#define DEFAULT_CONTROL_RATE        1.0f

#define ATTITUDE_DISARM_RANGE_MAX   10.0f
#define ATTITUDE_DISARM_RANGE_MIN   -10.0f

#define THROTTLE_CHANGE_RATE        50   /* unit value per ms */

#define DEFAULT_CONTROL_ATT_RANGE   25.0f
#define DEFAULT_CONTROL_GYR_RANGE   500.0f

#define MAX_CONTROL_RATE            1.5f
#define MIN_CONTROL_RATE            0.5f
#define MAX_CONTROL_ATT_RANGE       45.0f
#define MIN_CONTROL_ATT_RANGE       15.0f
#define MAX_CONTROL_GYR_RANGE       800.0f
#define MIN_CONTROL_GYR_RANGE       200.0f

#define Check_Control_Rate(x)       ((x > MAX_CONTROL_RATE) ? MAX_CONTROL_RATE : ((x < MIN_CONTROL_RATE) ? MIN_CONTROL_RATE : x))
#define Check_AttControl_Range(x)   ((x > MAX_CONTROL_ATT_RANGE) ? MAX_CONTROL_ATT_RANGE : ((x < MIN_CONTROL_ATT_RANGE) ? MIN_CONTROL_ATT_RANGE : x))
#define Check_GyrControl_Range(x)   ((x > MAX_CONTROL_GYR_RANGE) ? MAX_CONTROL_GYR_RANGE : ((x < MIN_CONTROL_GYR_RANGE) ? MIN_CONTROL_GYR_RANGE : x))

static uint32_t imu_update_time = 0;
static uint32_t att_update_time = 0;
static uint8_t imu_err_code;
static bool failsafe = false;
static bool imu_init_state = false;
static bool att_update = false;

DataPipe_CreateDataObj(ExpControlData_TypeDef, ExpCtl);

TaskControl_Monitor_TypeDef TaskControl_Monitor = {
    .init_state = false,
    .control_abort = false,

    /* on test mode use angular_speed over rate threshold protect */
    .angular_protect_enable = true,
    .angular_protect = false,

    /* on test mode for throttle control value mutation protect */
    .throttle_protect_enable = true,
    .throttle_protect = false,
    .throttle_percent = 0,

    .dynamic_disarm_enable = false,
    .IMU_Rt = 0,
};

/* internal function */
static void TaskControl_FlightControl_Polling(ControlData_TypeDef *exp_ctl_val, uint32_t sys_ms);
static void TaskControl_Actuator_ControlValue_Update(uint16_t throttle, float GX_Ctl_Out, float GY_Ctl_Out, float GZ_Ctl_Out);
static void TaskControl_CLI_Polling(void);
static void TaskControl_Get_StoreParam(void);

/* internal var */
static uint32_t TaskControl_Period = 0;

void TaskControl_Init(uint32_t period)
{
    /* init monitor */
    memset(&TaskControl_Monitor, 0, sizeof(TaskControl_Monitor));
    SrvDataHub.get_imu_init_state(&imu_init_state);

    /* pipe init */
    CtlData_smp_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(ExpCtl);
    CtlData_smp_DataPipe.data_size = DataPipe_DataSize(ExpCtl);
    DataPipe_Enable(&CtlData_smp_DataPipe);
    
    /* Parametet Init */
    TaskControl_Get_StoreParam();
    TaskControl_Monitor.init_state = SrvActuator.init(TaskControl_Monitor.actuator_param);

    osMessageQDef(MotoCLI_Data, 64, TaskControl_CLIData_TypeDef);
    TaskControl_Monitor.CLIMessage_ID = osMessageCreate(osMessageQ(MotoCLI_Data), NULL);
    
    TaskControl_Period = period;
}

/* read param from storage */
static void TaskControl_Get_StoreParam(void)
{
    SrvActuator_Setting_TypeDef Actuator_Param;
    TaskControl_CtlPara_TypeDef Ctl_Param;

    /* search storage section first */
    memset(&Ctl_Param, 0, sizeof(TaskControl_CtlPara_TypeDef));
    memset(&Actuator_Param, 0, sizeof(SrvActuator_Setting_TypeDef));
    memset(&TaskControl_Monitor.controller_info, 0, sizeof(Storage_ItemSearchOut_TypeDef));
    memset(&TaskControl_Monitor.actuator_store_info, 0, sizeof(Storage_ItemSearchOut_TypeDef));

    TaskControl_Monitor.controller_info = Storage.search(Para_User, CONTROL_STORAGE_SECTION_NAME);
    TaskControl_Monitor.actuator_store_info = Storage.search(Para_User, ACTUATOR_STORAGE_SECTION_NAME);

    /* set Ctl Parameter as default */
    Ctl_Param.att_mode = DEFAULT_ATTITUDE_CONTROLLER_MODE;
    Ctl_Param.alt_mode = DEFAULT_ATTITUDE_CONTROLLER_MODE;
    Ctl_Param.att_rate = DEFAULT_CONTROL_RATE;
    Ctl_Param.gx_rate = DEFAULT_CONTROL_RATE;
    Ctl_Param.gy_rate = DEFAULT_CONTROL_RATE;
    Ctl_Param.gz_rate = DEFAULT_CONTROL_RATE;
    Ctl_Param.pitch_range = DEFAULT_CONTROL_ATT_RANGE;
    Ctl_Param.roll_range = DEFAULT_CONTROL_ATT_RANGE;
    Ctl_Param.gx_range = DEFAULT_CONTROL_GYR_RANGE;
    Ctl_Param.gy_range = DEFAULT_CONTROL_GYR_RANGE;
    Ctl_Param.gz_range = DEFAULT_CONTROL_GYR_RANGE;
    memcpy(&TaskControl_Monitor.ctl_para, &Ctl_Param, sizeof(TaskControl_CtlPara_TypeDef));

    if (TaskControl_Monitor.controller_info.item_addr == 0)
    {
        /* section create */
        Storage.create(Para_User, CONTROL_STORAGE_SECTION_NAME, (uint8_t *)&Ctl_Param, sizeof(TaskControl_CtlPara_TypeDef));
    }
    else if (Storage.get(Para_User, TaskControl_Monitor.controller_info.item, (uint8_t *)&Ctl_Param, sizeof(TaskControl_CtlPara_TypeDef)) == Storage_Error_None)
    {
        /* check rate parameter validation */
        Ctl_Param.att_rate = Check_Control_Rate(Ctl_Param.att_rate);
        Ctl_Param.gx_rate = Check_Control_Rate(Ctl_Param.gx_rate);
        Ctl_Param.gy_rate = Check_Control_Rate(Ctl_Param.gy_rate);
        Ctl_Param.gz_rate = Check_Control_Rate(Ctl_Param.gz_rate);

        /* check range parameter validation */
        Ctl_Param.pitch_range = Check_AttControl_Range(Ctl_Param.pitch_range);
        Ctl_Param.roll_range = Check_AttControl_Range(Ctl_Param.roll_range);
        Ctl_Param.gx_range = Check_GyrControl_Range(Ctl_Param.gx_range);
        Ctl_Param.gy_range = Check_GyrControl_Range(Ctl_Param.gy_range);
        Ctl_Param.gz_range = Check_GyrControl_Range(Ctl_Param.gz_range);

        memcpy(&TaskControl_Monitor.ctl_para, &Ctl_Param, sizeof(TaskControl_CtlPara_TypeDef));
    }
    
    /* controller load parameter from storage */
    Controller.att_ctl_init(TaskControl_Monitor.ctl_para.att_mode);
    Controller.alt_ctl_init(TaskControl_Monitor.ctl_para.alt_mode);

    /* get actuator parameter */
    /* set as default */
    TaskControl_Monitor.actuator_param = SrvActuator.default_param();
    if (TaskControl_Monitor.actuator_store_info.item_addr == 0)
    {
        Storage.create(Para_User, ACTUATOR_STORAGE_SECTION_NAME, (uint8_t *)&TaskControl_Monitor.actuator_param, sizeof(SrvActuator_Setting_TypeDef));
    }
    else if (TaskControl_Monitor.actuator_store_info.item_addr && \
             (Storage.get(Para_User, TaskControl_Monitor.actuator_store_info.item, (uint8_t *)&Actuator_Param, sizeof(SrvActuator_Setting_TypeDef)) == Storage_Error_None))
        TaskControl_Monitor.actuator_param = Actuator_Param;
}

static bool TaskControl_disarm_check(bool telemetry_arm, float pitch, float roll)
{
    if (telemetry_arm == DRONE_ARM)
    {
        TaskControl_Monitor.moto_unlock = Moto_Lock;
        return false;
    }

    if (TaskControl_Monitor.dynamic_disarm_enable)
    {
        TaskControl_Monitor.moto_unlock = Moto_Unlock;
        return true;
    }

    if (TaskControl_Monitor.moto_unlock != Moto_Unlock)
    {
        if (TaskControl_Monitor.moto_unlock == Moto_Unlock_Err)
            return false;

        /* attitude pitch check */
        if ((pitch > ATTITUDE_DISARM_RANGE_MAX) || \
            (pitch < ATTITUDE_DISARM_RANGE_MIN) || \
            (roll > ATTITUDE_DISARM_RANGE_MAX) || \
            (roll < ATTITUDE_DISARM_RANGE_MIN))
        {
            TaskControl_Monitor.moto_unlock = Moto_Unlock_Err;
            return false;
        }

        TaskControl_Monitor.moto_unlock = Moto_Unlock;
        return true;
    }

    return true;
}

void TaskControl_Core(void const *arg)
{
    uint32_t sys_time = SrvOsCommon.get_os_ms();
    uint32_t task_tick = 0;
    bool upgrade_state = false;
    ControlData_TypeDef CtlData;
    memset(&CtlData, 0, sizeof(ControlData_TypeDef));

    while(1)
    {
        task_tick = SrvOsCommon.get_os_ms();

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
                /* debug set control to angular speed control */
                TaskControl_FlightControl_Polling(&CtlData, task_tick);
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

static void TaskControl_Actuator_ControlValue_Update(uint16_t throttle, float GX_Ctl_Out, float GY_Ctl_Out, float GZ_Ctl_Out)
{
    int16_t ctl_buf[Actuator_Ctl_Sum] = {0};

    /* idle spin when disarm */
    ctl_buf[Actuator_Ctl_Throttle] = throttle;

    ctl_buf[Actuator_Ctl_GyrX] = (int16_t)GX_Ctl_Out;
    ctl_buf[Actuator_Ctl_GyrY] = (int16_t)GY_Ctl_Out;
    ctl_buf[Actuator_Ctl_GyrZ] = (int16_t)GZ_Ctl_Out;

    SrvActuator.moto_control(ctl_buf);
}

/****************************************************** Flight Control Section ********************************************************/
static float TaskControl_Convert_CtlData(uint8_t gimbal_percent, float range, float rate)
{
    float exp = 0.0f;

    exp = (gimbal_percent - 50.0f) / 50.0f;
    if (exp >= 0)
    {
        exp *= range;
    }
    else
        exp *= -range;
    
    return exp *= rate;
}

/* need to be optmize */
static void TaskControl_FlightControl_Polling(ControlData_TypeDef *exp_ctl_val, uint32_t sys_ms)
{
    uint8_t axis = Axis_X;
    bool arm_state = false;
    bool USB_Attach = false;
    AngControl_Out_TypeDef att_ctl_out;
    AttControl_In_TypeDef att_ctl_exp;
    AttControl_In_TypeDef att_ctl_mea;
    bool ang_ctl_only = false;

    memset(&att_ctl_exp, 0, sizeof(AttControl_In_TypeDef));
    memset(&att_ctl_out, 0, sizeof(AngControl_Out_TypeDef));

    if (TaskControl_Monitor.init_state && exp_ctl_val)
    {
        if (exp_ctl_val->update_time_stamp && !exp_ctl_val->fail_safe)
        {
            att_ctl_exp.pitch  = TaskControl_Convert_CtlData(exp_ctl_val->pitch_percent, TaskControl_Monitor.ctl_para.pitch_range, TaskControl_Monitor.ctl_para.att_rate);
            att_ctl_exp.roll   = TaskControl_Convert_CtlData(exp_ctl_val->roll_percent,  TaskControl_Monitor.ctl_para.roll_range,  TaskControl_Monitor.ctl_para.att_rate);
            att_ctl_exp.gyro_x = TaskControl_Convert_CtlData(exp_ctl_val->roll_percent, TaskControl_Monitor.ctl_para.gx_range,    TaskControl_Monitor.ctl_para.gx_rate);
            att_ctl_exp.gyro_y = TaskControl_Convert_CtlData(exp_ctl_val->pitch_percent,  TaskControl_Monitor.ctl_para.gy_range,    TaskControl_Monitor.ctl_para.gy_rate);
            att_ctl_exp.gyro_z = TaskControl_Convert_CtlData(exp_ctl_val->yaw_percent,   TaskControl_Monitor.ctl_para.gz_range,    TaskControl_Monitor.ctl_para.gz_rate);

            if (exp_ctl_val->control_mode == AngularSpeed_Control)
                ang_ctl_only = true;
        }

        arm_state = exp_ctl_val->arm_state;
        failsafe = exp_ctl_val->fail_safe;
        TaskControl_Monitor.control_abort = false;

        /* pipe converted control data to data hub */
        DataPipe_DataObj(ExpCtl).arm = arm_state;
        DataPipe_DataObj(ExpCtl).failsafe = failsafe;
        DataPipe_DataObj(ExpCtl).throttle_percent = exp_ctl_val->throttle_percent;
        DataPipe_DataObj(ExpCtl).control_mode = exp_ctl_val->control_mode;
        DataPipe_DataObj(ExpCtl).pitch = att_ctl_exp.pitch;
        DataPipe_DataObj(ExpCtl).roll  = att_ctl_exp.roll;
        DataPipe_DataObj(ExpCtl).gyr_x = att_ctl_exp.gyro_x;
        DataPipe_DataObj(ExpCtl).gyr_y = att_ctl_exp.gyro_y;
        DataPipe_DataObj(ExpCtl).gyr_z = att_ctl_exp.gyro_z;
        DataPipe_SendTo(&CtlData_smp_DataPipe, &CtlData_hub_DataPipe);
        DataPipe_SendTo(&CtlData_smp_DataPipe, &CtlData_Log_DataPipe);

        // check imu filter gyro data update or not
        if(!SrvDataHub.get_scaled_imu(&imu_update_time,
                                      NULL, NULL,
                                      &TaskControl_Monitor.acc[Axis_X],
                                      &TaskControl_Monitor.acc[Axis_Y],
                                      &TaskControl_Monitor.acc[Axis_Z],
                                      &TaskControl_Monitor.gyr[Axis_X],
                                      &TaskControl_Monitor.gyr[Axis_Y],
                                      &TaskControl_Monitor.gyr[Axis_Z],
                                      NULL, &imu_err_code) || !imu_init_state)
            goto lock_moto;

        /* if angular speed over ride then lock the moto and set drone as arm */

        // get attitude
        att_update = SrvDataHub.get_attitude(&att_update_time,
                                             &TaskControl_Monitor.pitch,
                                             &TaskControl_Monitor.roll,
                                             &TaskControl_Monitor.yaw,
                                             NULL, NULL, NULL, NULL,
                                             &TaskControl_Monitor.flip_over);

        att_ctl_mea.pitch  = TaskControl_Monitor.pitch;
        att_ctl_mea.roll   = TaskControl_Monitor.roll;
        att_ctl_mea.gyro_x = TaskControl_Monitor.gyr[Axis_X];
        att_ctl_mea.gyro_y = TaskControl_Monitor.gyr[Axis_Y];
        att_ctl_mea.gyro_z = TaskControl_Monitor.gyr[Axis_Z];

        TaskControl_disarm_check(arm_state, TaskControl_Monitor.pitch, TaskControl_Monitor.roll);

        /* if armed or usb attached then lock moto */
        if (TaskControl_Monitor.moto_unlock != Moto_Unlock)
            goto lock_moto;

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

            /* Controller Update */
            /* altitude control update */
            /* attitude control update */
            Controller.att_ctl(TaskControl_Monitor.ctl_para.att_mode, sys_ms, ang_ctl_only, att_ctl_exp, att_ctl_mea, &att_ctl_out);

            /* when when usb attached lock moto */
            if (SrvDataHub.get_vcp_attach_state(&USB_Attach) || !USB_Attach)
            {
                TaskControl_Actuator_ControlValue_Update(TaskControl_Monitor.throttle_percent, \
                                                         att_ctl_out.gyro_x, \
                                                         att_ctl_out.gyro_y, \
                                                         att_ctl_out.gyro_z);

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
    }

lock_moto:
    SrvActuator.lock();
}

/****************************************************** CLI Section ******************************************************************/
static void TaskControl_CLI_Polling(void)
{
    osEvent event;
    static TaskControl_CLIData_TypeDef CLIData;
    static uint16_t moto_ctl_buff[8] = {0};
    Shell *shell_obj = Shell_GetInstence();

    if ((shell_obj == NULL) || \
        (SrvActuator.lock == NULL) || \
        (SrvActuator.moto_direct_drive == NULL))
        return;

    if(TaskControl_Monitor.CLIMessage_ID)
    {
        event = osMessageGet(TaskControl_Monitor.CLIMessage_ID, CLI_MESSAGE_OPEARATE_TIMEOUT);
        if (event.status == osEventMessage)
        {
            if (event.value.p == NULL)
            {
                SrvActuator.lock();
                return;
            }

            CLIData = *(TaskControl_CLIData_TypeDef *)(event.value.p);
            switch((uint8_t)CLIData.cli_type)
            {
                case TaskControl_Moto_Set_SpinDir:
                    memset(moto_ctl_buff, 0, sizeof(moto_ctl_buff));
                    if(SrvActuator.set_spin_dir(CLIData.index, (uint8_t)CLIData.value))
                    {
                        shellPrint(shell_obj, "moto spin dir set done\r\n");
                        SrvActuator.save(CLIData.index);
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
        for (uint8_t i = 0; i < SrvActuator.get_cnt().moto_cnt; i++)
            SrvActuator.moto_direct_drive(i, moto_ctl_buff[i]);
    }
    else
        SrvActuator.lock();
}

static void TaskControl_CLI_ShowModleInfo(void)
{
    Shell *shell_obj = Shell_GetInstence();
    if (shell_obj == NULL)
        return;

    shellPrint(shell_obj, "[ control parameter ]\r\n");
    shellPrint(shell_obj, "--- control rate ---\r\n");
    shellPrint(shell_obj, "    attitude        control rate %f\r\n", TaskControl_Monitor.ctl_para.att_rate);
    shellPrint(shell_obj, "    X angular speed control rate %f\r\n", TaskControl_Monitor.ctl_para.gx_rate);
    shellPrint(shell_obj, "    Y angular speed control rate %f\r\n", TaskControl_Monitor.ctl_para.gy_rate);
    shellPrint(shell_obj, "    Z angular speed control rate %f\r\n", TaskControl_Monitor.ctl_para.gz_rate);
    
    shellPrint(shell_obj, "--- control range ---\r\n");
    shellPrint(shell_obj, "    pitch           range ±%d\r\n", TaskControl_Monitor.ctl_para.pitch_range);
    shellPrint(shell_obj, "    roll            range ±%d\r\n", TaskControl_Monitor.ctl_para.roll_range);
    shellPrint(shell_obj, "    X angular speed range ±%d\r\n", TaskControl_Monitor.ctl_para.gx_range);
    shellPrint(shell_obj, "    Y angular speed range ±%d\r\n", TaskControl_Monitor.ctl_para.gy_range);
    shellPrint(shell_obj, "    Z angular speed range ±%d\r\n", TaskControl_Monitor.ctl_para.gz_range);
    shellPrint(shell_obj, "--- attitude control mode ---\r\n");
    switch (TaskControl_Monitor.ctl_para.att_mode)
    {
        case CtlM_PID:   shellPrint(shell_obj, " ---- pid\r\n"); break;
        case CtlM_LADRC: shellPrint(shell_obj, " ---- ladrc\r\n"); break;
        case CtlM_MUDE:  shellPrint(shell_obj, " ---- mude\r\n"); break;
        default: shellPrint(shell_obj, " ---- unknow control mode\r\n"); break;
    }

    shellPrint(shell_obj, "--- altitude control mode ---\r\n");
    switch (TaskControl_Monitor.ctl_para.alt_mode)
    {
        case CtlM_PID:   shellPrint(shell_obj, " ---- pid\r\n"); break;
        case CtlM_LADRC: shellPrint(shell_obj, " ---- ladrc\r\n"); break;
        case CtlM_MUDE:  shellPrint(shell_obj, " ---- mude\r\n"); break;
        default: shellPrint(shell_obj, " ---- unknow control mode\r\n"); break;
    }

    shellPrint(shell_obj, "[ actuator parameter ]\r\n");
    shellPrint(shell_obj, "    moto  num %d\r\n", TaskControl_Monitor.actuator_param.moto_num);
    shellPrint(shell_obj, "    servo num %d\r\n", TaskControl_Monitor.actuator_param.servo_num);
    switch (TaskControl_Monitor.actuator_param.esc_type)
    {
        case Actuator_DevType_DShot150: shellPrint(shell_obj, " ---- ESC DShot_150\r\n"); break;
        case Actuator_DevType_DShot300: shellPrint(shell_obj, " ---- ESC DShot_300\r\n"); break;
        case Actuator_DevType_DShot600: shellPrint(shell_obj, " ---- ESC DShot_600\r\n"); break;
        case Actuator_DevType_Brush:    shellPrint(shell_obj, " ---- ESC Brush\r\n"); break;
        default: break;
    }
    
    switch (TaskControl_Monitor.actuator_param.model)
    {
        case Model_Quad:   shellPrint(shell_obj, " ---- QUAD\r\n"); break;
        case Model_Hex:    shellPrint(shell_obj, " ---- HEX\r\n"); break;
        case Model_Y6:     shellPrint(shell_obj, " ---- Y6\r\n"); break;
        case Model_Tri:    shellPrint(shell_obj, " ---- TRI\r\n"); break;
        case Model_TDrone: shellPrint(shell_obj, " ---- TDrone\r\n"); break;
        default: break;
    }

    shellPrint(shell_obj, "---- moto mapping relationship ----\r\n");
    for (uint8_t i = 0; i < TaskControl_Monitor.actuator_param.moto_num; i ++)
    {
        shellPrint(shell_obj, "    moto list index %d -> map to phy rotor %d\r\n", i, TaskControl_Monitor.actuator_param.moto_map[i]);
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, show_ctl_modle, TaskControl_CLI_ShowModleInfo, show modle info);

static void TaskControl_CLI_SetMotoType(uint8_t type)
{
    Shell *shell_obj = Shell_GetInstence();
    SrvActuator_Setting_TypeDef actuator_para_tmp = TaskControl_Monitor.actuator_param;

    if (shell_obj == NULL)
        return;

    shellPrint(shell_obj, "ESC type list\r\n");
    shellPrint(shell_obj, " %d ---- DShot150 \r\n", Actuator_DevType_DShot150);
    shellPrint(shell_obj, " %d ---- DShot300 \r\n", Actuator_DevType_DShot300);
    shellPrint(shell_obj, " %d ---- DShot600 \r\n", Actuator_DevType_DShot600);
    shellPrint(shell_obj, " %d ---- Brush \r\n", Actuator_DevType_Brush);

    switch (type)
    {
        case Actuator_DevType_DShot300: shellPrint(shell_obj, "    DShot300 Selected\r\n"); break;
        case Actuator_DevType_Brush:    shellPrint(shell_obj, "    BrushMoto Selected\r\n"); break;
        default: shellPrint(shell_obj, "Error ESC type or none support ESC type input\r\n"); return;
    }

    actuator_para_tmp.esc_type = type;
    if (Storage.update(Para_User, TaskControl_Monitor.actuator_store_info.item_addr, (uint8_t *)&actuator_para_tmp, sizeof(SrvActuator_Setting_TypeDef)) != Storage_Error_None)
    {
        shellPrint(shell_obj, "Parameter update failed\r\n");
        return;
    }

    /* after setting reboot is required */
    shellPrint(shell_obj, "Parameter update done\r\n");
    shellPrint(shell_obj, "rebooting\r\n");

    SrvOsCommon.delay_ms(200);
    SrvOsCommon.reboot();
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, set_moto_type, TaskControl_CLI_SetMotoType, set moto type);

static void TaskControl_CLI_MotoSpinTest(uint8_t moto_index, uint16_t test_val)
{
    uint8_t moto_num = SrvActuator.get_cnt().moto_cnt;
    bool arm_state = false;
    int16_t moto_max = 0;
    int16_t moto_idle = 0;
    int16_t moto_min = 0;
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
                break;
            }
            
            shellPrint(shell_obj, "moto %d control range down below\r\n", i);
            shellPrint(shell_obj, "\tmax  : %d\r\n", moto_max);
            shellPrint(shell_obj, "\tidle : %d\r\n", moto_idle);
            shellPrint(shell_obj, "\tmin  : %d\r\n", moto_min);
        }
    }
    else
    {
        shellPrint(shell_obj, "moto %d is selected\r\n", moto_index);
        if(!SrvActuator.get_moto_control_range(moto_index, &moto_min, &moto_idle, &moto_max))
            ctl_enable = false;

        shellPrint(shell_obj, "control range down below\r\n");
        shellPrint(shell_obj, "\tmax  : %d\r\n", moto_max);
        shellPrint(shell_obj, "\tidle : %d\r\n", moto_idle);
        shellPrint(shell_obj, "\tmin  : %d\r\n", moto_min);
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
            p_CLIData->timestamp = SrvOsCommon.get_os_ms();
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

static void TaskControl_Reverse_SpinDir(uint8_t moto_index, uint8_t dir)
{
    Shell *shell_obj = Shell_GetInstence();
    uint8_t moto_num = SrvActuator.get_cnt().moto_cnt;
    TaskControl_CLIData_TypeDef *p_CLIData = NULL;
    bool arm_state = false;

    if (shell_obj == NULL)
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

    if((moto_index >= moto_num) || (dir > 2))
    {
        shellPrint(shell_obj, "parameter error\r\n");
        return;
    }

    shellPrint(shell_obj, "moto %d is selected dir %d\r\n", moto_index, dir);
    p_CLIData = SrvOsCommon.malloc(sizeof(TaskControl_CLIData_TypeDef));
    if(p_CLIData)
    {
        p_CLIData->cli_type = TaskControl_Moto_Set_SpinDir;
        p_CLIData->index = moto_index;
        p_CLIData->timestamp = SrvOsCommon.get_os_ms();
        p_CLIData->value = dir;

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
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, rev_spin, TaskControl_Reverse_SpinDir, reverse moto Spin);

static void TaskControl_Close_CLI(void)
{
    Shell *shell_obj = Shell_GetInstence();
    TaskControl_CLIData_TypeDef *p_CLIData = NULL;
    uint32_t time_stamp = SrvOsCommon.get_os_ms();

    if(shell_obj == NULL)
        return;

    if(TaskControl_Monitor.CLIMessage_ID == NULL)
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
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, lock_moto, TaskControl_Close_CLI, disable actuator test);

