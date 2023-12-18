/*
 *  coder : 8_B!T0
 *  this file use for moto & servo control
 */
#include "cmsis_os.h"
#include "Task_Control.h"
#include "Srv_OsCommon.h"
#include "Srv_CtlDataArbitrate.h"
#include "Task_Telemetry.h"
#include "Srv_DataHub.h"
#include "Srv_Actuator.h"
#include "shell_port.h"

#define DEFAULT_CONTROL_MODEL Model_Quad
#define DEFAULT_ESC_TYPE DevDshot_600

#define ATTITUDE_PID_ACCURACY 1000
#define ATTITUDE_PID_DIFF_MAX 30    /* unit: deg */
#define ATTITUDE_PID_DIFF_MIN -30   /* unit: deg */

#define ANGULAR_PID_ACCURACY 1000

static uint32_t rc_update_time = 0;
static uint32_t imu_update_time = 0;
static uint32_t att_update_time = 0;
static uint32_t tunning_time_stamp = 0;
static uint16_t rc_ch[32];
static uint16_t gimbal[4];
static uint8_t rc_channel_sum;
static uint8_t imu_err_code;
static uint8_t axis = Axis_X;
static uint32_t tunning_port = 0;
static bool arm_state = true;
static bool failsafe = false;
static bool imu_init_state = false;
static bool att_update = false;
static bool rc_update = false;
static bool tunning_state = false;
static bool configrator_attach = false;

SrvIMU_UnionData_TypeDef LstCyc_IMU_Data;
SrvRecever_RCSig_TypeDef LstCyc_Rc_Data;

TaskControl_Monitor_TypeDef TaskControl_Monitor = {
    .init_state = false,
    .control_abort = false,
    .auto_control = false,

    .ctl_model = Model_Quad,

    .IMU_Rt = 0,
    .RC_Rt = 0,
};

/* internal function */
static bool TaskControl_AttitudeRing_PID_Update(TaskControl_Monitor_TypeDef *monitor, bool att_state);
static bool TaskControl_AngularSpeedRing_PID_Update(TaskControl_Monitor_TypeDef *monitor);
static void TaskControl_FlightControl_Polling(void);
static void TaskControl_CLI_Polling(void);

/* internal var */
static uint32_t TaskControl_Period = 0;

void TaskControl_Init(uint32_t period)
{
    // init monitor
    memset(&TaskControl_Monitor, 0, sizeof(TaskControl_Monitor));

    TaskControl_Monitor.ctl_model = SrvActuator.get_model();
    TaskControl_Monitor.init_state = SrvActuator.init(DEFAULT_CONTROL_MODEL, DEFAULT_ESC_TYPE);

    /* PID Parametet Init */
    TaskControl_Monitor.PitchCtl_PIDObj.accuracy_scale = ATTITUDE_PID_ACCURACY;
    TaskControl_Monitor.RollCtl_PIDObj.accuracy_scale  = ATTITUDE_PID_ACCURACY;
    TaskControl_Monitor.YawCtl_PIDObj.accuracy_scale   = ATTITUDE_PID_ACCURACY;

    TaskControl_Monitor.PitchCtl_PIDObj.diff_max       = ATTITUDE_PID_DIFF_MAX;
    TaskControl_Monitor.RollCtl_PIDObj.diff_max        = ATTITUDE_PID_DIFF_MAX;

    TaskControl_Monitor.PitchCtl_PIDObj.diff_min       = ATTITUDE_PID_DIFF_MIN;
    TaskControl_Monitor.RollCtl_PIDObj.diff_min        = ATTITUDE_PID_DIFF_MIN;
    
    TaskControl_Monitor.GyrXCtl_PIDObj.accuracy_scale = ANGULAR_PID_ACCURACY;
    TaskControl_Monitor.GyrYCtl_PIDObj.accuracy_scale = ANGULAR_PID_ACCURACY;
    TaskControl_Monitor.GyrZCtl_PIDObj.accuracy_scale = ANGULAR_PID_ACCURACY;

    osMessageQDef(MotoCLI_Data, 64, TaskControl_CLIData_TypeDef);
    TaskControl_Monitor.CLIMessage_ID = osMessageCreate(osMessageQ(MotoCLI_Data), 64, NULL);

    TaskControl_Period = period;
}

void TaskControl_Core(void const *arg)
{
    uint32_t sys_time = SrvOsCommon.get_os_ms();

    while(1)
    {
        SrvDataHub.get_arm_state(&arm_state);
        
        if((DRONE_DISARM == arm_state) && !TaskControl_Monitor.CLI_enable)
        {
            TaskControl_FlightControl_Polling();
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

            return true;
        }
    }

    return false;
}

static bool TaskControl_AngularSpeedRing_PID_Update(TaskControl_Monitor_TypeDef *monitor)
{
    if(monitor)
    {
        if(monitor->att_pid_state)
        {

        }

        return true;
    }

    return false;
}

/****************************************************** Flight Control Section ********************************************************/
static void TaskControl_FlightControl_Polling(void)
{
    if (TaskControl_Monitor.init_state && !TaskControl_Monitor.control_abort)
    {
        TaskControl_Monitor.auto_control = true;
        imu_init_state = false;

        // get failsafe
        SrvDataHub.get_arm_state(&arm_state);
        SrvDataHub.get_failsafe(&failsafe);
        SrvDataHub.get_gimbal_percent(gimbal);
        
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

        // get attitude
        att_update = SrvDataHub.get_attitude(&att_update_time,
                                             &TaskControl_Monitor.attitude.pitch,
                                             &TaskControl_Monitor.attitude.roll,
                                             &TaskControl_Monitor.attitude.yaw,
                                             &TaskControl_Monitor.attitude.q0,
                                             &TaskControl_Monitor.attitude.q1,
                                             &TaskControl_Monitor.attitude.q2,
                                             &TaskControl_Monitor.attitude.q3);

        // get rc channel and other toggle signal
        rc_update = SrvDataHub.get_rc(&rc_update_time, rc_ch, &rc_channel_sum);

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

        /* only manipulate esc or servo when disarm */
        if (rc_update_time && !failsafe)
        {
            if (rc_update_time >= TaskControl_Monitor.RC_Rt)
            {
                TaskControl_Monitor.auto_control = false;
                TaskControl_Monitor.RC_Rt = rc_update_time;

                if (arm_state != TELEMETRY_SET_DISARM)
                    goto lock_moto;
            }
        }
        
        if(TaskControl_Monitor.angular_protect)
            goto lock_moto;

        /* currently lock moto */
        if(TaskControl_Monitor.auto_control)
            goto lock_moto;

        // do drone control algorithm down below

        // currently use gimbal input percent val for moto testing
        gimbal[1] = 0;
        gimbal[2] = 0;
        gimbal[3] = 0;

        /* Update PID */
        TaskControl_AttitudeRing_PID_Update(&TaskControl_Monitor, att_update);
        TaskControl_AngularSpeedRing_PID_Update(&TaskControl_Monitor);

        SrvActuator.moto_control(gimbal);

        if(imu_err_code == SrvIMU_Sample_NoError)
        {
            for(axis = Axis_X; axis < Axis_Sum; axis ++)
            {
                TaskControl_Monitor.acc_lst[axis] = TaskControl_Monitor.acc[axis];
                TaskControl_Monitor.gyr_lst[axis] = TaskControl_Monitor.gyr[axis];
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
    TaskControl_CLIData_TypeDef *p_CLIData = NULL;
    static int16_t moto_ctl_val = 0;
    uint8_t moto_sum = SrvActuator.get_cnt().moto_cnt;
    uint8_t servo_sum = SrvActuator.get_cnt().servo_cnt;
    uint16_t moto_ctl_buff[moto_sum] = {0};
    uint16_t servo_ctl_buff[servo_sum] = {0};
    Shell *shell_obj = Shell_GetInstence();

    if(shell_obj == NULL)
        return;

    if(TaskControl_Monitor.CLIMessage_ID)
    {
        event = osMessageGet(TaskControl_Monitor.CLIMessage_ID, CLI_MESSAGE_OPEARATE_TIMEOUT);
    
        if(event.def.message_id == TaskControl_Monitor.CLIMessage_ID)
        {
            switch(event.status)
            {
                case osOK:
                    p_CLIData = event.value.p;

                    switch((uint8_t)p_CLIData->cli_type)
                    {
                        case TaskControl_Moto_Set_Spin:
                            if(SrvActuator.invert_spin(p_CLIData->index))
                            {
                                shellPrint(shell_obj, "moto spin dir set done\r\n");
                            }
                            else
                                shellPrint(shell_obj, "moto spin dir set error\r\n");
                            break;

                        case TaskControl_Moto_Set_SpinDir:
                            if(p_CLIData->index < SrvActuator.get_cnt().moto_cnt)
                            {
                                moto_ctl_buff[p_CLIData->index] = p_CLIData->value;
                            }
                            else
                            {
                                for(uint8_t i = 0; i < SrvActuator.get_cnt().moto_cnt; i++)
                                {
                                    moto_ctl_buff[i] = p_CLIData->value;
                                }
                            }
                            break;

                        default:
                            break;
                    }

                    SrvOsCommon.free(p_CLIData);
                    break;

                case osEventSignal:
                case osEventMessage:
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
    }

    if(moto_ctl_val)
    {
        SrvActuator.moto_control(moto_ctl_val);
    }
    else
        SrvActuator.lock();
}

static void TaskControl_CLI_AllMotoSpinTest(uint16_t test_val)
{
    bool arm_state = false;
    uint32_t time_stamp = SrvOsCommon.get_os_ms();
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

                            if(osMessagePut(TaskControl_Monitor.CLIMessage_ID, p_CLIData, CLI_MESSAGE_OPEARATE_TIMEOUT) != osOK)
                                shellPrint(shell_obj, "TaskControl CLI set failed\r\n");
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

                    if(osMessagePut(TaskControl_Monitor.CLIMessage_ID, p_CLIData, CLI_MESSAGE_OPEARATE_TIMEOUT) != osOK)
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
    uint32_t time_stamp = SrvOsCommon.get_os_ms();
    Shell *shell_obj = Shell_GetInstence();

    if(shell_obj == NULL)
        return;

    if(TaskControl_Monitor.CLIMessage_ID)
    {
        shellPrint(shell_obj, "TaskControl CLI disable\r\n");
        SrvActuator.lock();
        TaskControl_Monitor.CLI_enable = false;
    }
    else
    {
        shellPrint(shell_obj, "TaskControl semaphore create failed\r\n");
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, disable_actuator_test, TaskControl_Close_CLI, disable actuator test);
