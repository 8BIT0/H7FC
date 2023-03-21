/*
 *  coder : 8_B!T0
 *  this file use for moto & servo control
 */
#include "Task_Control.h"
#include "Srv_IMUSample.h"
#include "Task_Telemetry.h"
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

    .IMU_Rt = 0,
    .RC_Rt = 0,
};

void TaskControl_Init(void)
{
    // init monitor
    memset(&TaskControl_Monitor, 0, sizeof(TaskControl_Monitor));

    TaskControl_Monitor.ctl_model = SrvActuator.get_model();
    TaskControl_Monitor.init_state = SrvActuator.init(DEFAULT_CONTROL_MODEL, DEFAULT_ESC_TYPE);
}

void TaskControl_Core(Task_Handle hdl)
{
    uint64_t imu_update_time = 0;
    uint32_t rc_update_time = 0;
    uint16_t rc_ch[32];
    uint16_t gimbal[4];
    uint8_t rc_channel_sum;
    bool arm_state;
    bool failsafe;

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

        // get rc channel and other toggle signal
        SrvDataHub.get_rc(&rc_update_time, rc_ch, &rc_channel_sum);

        // get failsafe
        SrvDataHub.get_arm_state(&arm_state);
        SrvDataHub.get_failsafe(&failsafe);
        SrvDataHub.get_gimbal(gimbal);

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
        if (rc_update_time)
        {
            if (!failsafe)
            {
                TaskControl_Monitor.RC_Rt = rc_update_time;

                if (arm_state != TELEMETRY_SET_DISARM)
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

            // currently use gimbal input val for moto testing
            SrvActuator.moto_control(gimbal);
        }
        else
            SrvActuator.lock();
    }
    else
        SrvActuator.lock();
}
