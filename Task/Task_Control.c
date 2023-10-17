/*
 *  coder : 8_B!T0
 *  this file use for moto & servo control
 */
#include "cmsis_os.h"
#include "Task_Control.h"
#include "Srv_OsCommon.h"
#include "Task_Telemetry.h"
#include "Srv_DataHub.h"
#include "Srv_Actuator.h"

#define DEFAULT_CONTROL_MODEL Model_Quad
#define DEFAULT_ESC_TYPE DevDshot_600

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

/* internal var */
static uint32_t TaskControl_Period = 0;

void TaskControl_Init(uint32_t period)
{
    // init monitor
    memset(&TaskControl_Monitor, 0, sizeof(TaskControl_Monitor));

    TaskControl_Monitor.ctl_model = SrvActuator.get_model();
    TaskControl_Monitor.init_state = SrvActuator.init(DEFAULT_CONTROL_MODEL, DEFAULT_ESC_TYPE);

    TaskControl_Period = period;
}

void TaskControl_Core(void const *arg)
{
    uint32_t sys_time = SrvOsCommon.get_os_ms();
    uint32_t imu_update_time = 0;
    uint32_t rc_update_time = 0;
    uint16_t rc_ch[32];
    uint16_t gimbal[4];
    uint8_t rc_channel_sum;
    uint8_t imu_err_code;
    uint8_t axis = Axis_X;
    bool arm_state;
    bool failsafe;

    while(1)
    {
        if (TaskControl_Monitor.init_state && !TaskControl_Monitor.control_abort)
        {
            TaskControl_Monitor.auto_control = true;

            // check imu filter gyro data update or not
            SrvDataHub.get_scaled_imu(&imu_update_time,
                                    &TaskControl_Monitor.acc_scale,
                                    &TaskControl_Monitor.gyr_scale,
                                    &TaskControl_Monitor.acc[Axis_X],
                                    &TaskControl_Monitor.acc[Axis_Y],
                                    &TaskControl_Monitor.acc[Axis_Z],
                                    &TaskControl_Monitor.gyr[Axis_X],
                                    &TaskControl_Monitor.gyr[Axis_Y],
                                    &TaskControl_Monitor.gyr[Axis_Z],
                                    &TaskControl_Monitor.imu_tmpr,
                                    &imu_err_code);

            // get rc channel and other toggle signal
            SrvDataHub.get_rc(&rc_update_time, rc_ch, &rc_channel_sum);

            // get failsafe
            SrvDataHub.get_arm_state(&arm_state);
            SrvDataHub.get_failsafe(&failsafe);
            SrvDataHub.get_gimbal_percent(gimbal);

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
                        case SrvIMU_Sample_Module_UnReady:
                        case SrvIMU_Sample_Data_Acc_Blunt:
                        case SrvIMU_Sample_Data_Gyr_Blunt:
                        case SrvIMU_Sample_Data_Acc_OverRange:
                        case SrvIMU_Sample_Data_Gyr_OverRange:
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
            {
                if(arm_state == TELEMETRY_SET_ARM)
                {
                    TaskControl_Monitor.angular_protect = false;
                    TaskControl_Monitor.angular_warning_cnt = 0;
                }
                else
                    goto lock_moto;
            }

            /* currently lock moto */
            if(TaskControl_Monitor.auto_control)
                goto lock_moto;

            // do drone control algorithm down below

            // currently use gimbal input percent val for moto testing
            gimbal[1] = 0;
            gimbal[2] = 0;
            gimbal[3] = 0;

            SrvActuator.moto_control(gimbal);

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

lock_moto:
        SrvActuator.lock();

        SrvOsCommon.precise_delay(&sys_time, TaskControl_Period);
    }
}

