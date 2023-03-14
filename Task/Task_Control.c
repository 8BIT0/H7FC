/*
 *  coder : 8_B!T0
 *  this file use for moto & servo control
 */
#include "Task_Control.h"
#include "Srv_IMUSample.h"
#include "Task_Telemetry.h"
#include "Srv_DataHub.h"
#include "Srv_DataHub.h"
#include "scheduler.h"
#include "Srv_Actuator.h"
#include "mmu.h"

#define DEFAULT_CONTROL_MODEL Model_Quad
#define DEFAULT_ESC_TYPE DevDshot_600

SrvIMU_UnionData_TypeDef LstCyc_IMU_Data;
SrvRecever_RCSig_TypeDef LstCyc_Rc_Data;

TaskControl_Monitor_TypeDef TaskControl_Monitor = {
    .init_state = false,
    .control_abort = false,
    .auto_control = false,

    .ctl_model = Model_Quad,
    .actuator_num = 0,

    .ctl_buff = NULL,

    .IMU_Rt = 0,
    .RC_Rt = 0,
};

void TaskControl_Init(void)
{
    // init monitor
    memset(&TaskControl_Monitor, 0, sizeof(TaskControl_Monitor));

    TaskControl_Monitor.ctl_model = SrvActuator.get_model();

    TaskControl_Monitor.init_state = SrvActuator.init(DEFAULT_CONTROL_MODEL, DEFAULT_ESC_TYPE);

    if (TaskControl_Monitor.init_state)
    {
        TaskControl_Monitor.actuator_num = SrvActuator.get_cnt().total_cnt;
        TaskControl_Monitor.ctl_buff = (uint16_t *)MMU_Malloc(sizeof(uint16_t) * TaskControl_Monitor.actuator_num);

        if (TaskControl_Monitor.ctl_buff == NULL)
        {
            MMU_Free(TaskControl_Monitor.ctl_buff);
            TaskControl_Monitor.init_state = false;
            return;
        }

        memset(TaskControl_Monitor.ctl_buff, 0, sizeof(uint16_t) * TaskControl_Monitor.actuator_num);
    }
}

void TaskControl_Core(Task_Handle hdl)
{
    uint64_t imu_update_time = 0;
    float imu_tmpr = 0.0f;
    float acc_scale = 0.0f;
    float gyr_scale = 0.0f;
    float acc_x = 0.0f;
    float acc_y = 0.0f;
    float acc_z = 0.0f;
    float gyr_x = 0.0f;
    float gyr_y = 0.0f;
    float gyr_z = 0.0f;

    if (TaskControl_Monitor.init_state || TaskControl_Monitor.control_abort)
    {
        // check imu filter gyro data update or not
        SrvDataHub.get_scaled_imu(&imu_update_time, 
                                  &acc_scale, &gyr_scale, 
                                  &acc_x, &acc_y, &acc_z, 
                                  &gyr_x, &gyr_y, &gyr_z, 
                                  &imu_tmpr);

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
        }

        /* only manipulate esc or servo when disarm */
        if (DataPipe_DataObj(Control_RC_Data).time_stamp)
        {
            if (!DataPipe_DataObj(Control_RC_Data).failsafe)
            {
                TaskControl_Monitor.RC_Rt = DataPipe_DataObj(Control_RC_Data).time_stamp;

                if (DataPipe_DataObj(Control_RC_Data).arm_state == TELEMETRY_SET_DISARM)
                {
                    for (uint8_t i = 0; i < TaskControl_Monitor.actuator_num; i++)
                    {
                        /* currently use this section for dshot test */
                        /* throttlr idle value check */
                        TaskControl_Monitor.ctl_buff[i] = DataPipe_DataObj(Control_RC_Data).gimbal_val[Telemetry_RC_Throttle];
                    }
                }
                else
                {
                    SrvActuator.lock();
                    return;
                }
            }
            else
            {
                // do drone return to liftoff spot or do auto control
                TaskControl_Monitor.auto_control = true;

                /* currently for test for safety */
                SrvActuator.lock();
                return;
            }

            // do drone control algorithm down below

            SrvActuator.control(TaskControl_Monitor.ctl_buff, TaskControl_Monitor.actuator_num);
        }
        else
            SrvActuator.lock();
    }
    else
        SrvActuator.lock();
}
