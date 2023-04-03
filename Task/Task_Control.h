#ifndef __TASK_CONTROL_H
#define __TASK_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "scheduler.h"
#include "imu_data.h"
#include "../common/util.h"

#define TASKCONTROL_SET_BIT(x) UTIL_SET_BIT(x)
#define IMU_ERROR_UPDATE_MAX_COUNT 10
#define IMU_NONE_UPDATE_THRESHOLD 10
#define OVER_ANGULAR_ACCELERATE_COUNT 5

typedef union
{
    struct
    {
        uint32_t rc : 1;
        uint32_t imu : 1;

        uint32_t reserve : 30;
    }Sec;

    uint32_t val;
}TaskControl_ErrReg_TypeDef;

typedef enum
{
    TaskControl_None_Error = 0,

    /* IMU Section */
    TaskControl_IMU_NoneUpdate = TASKCONTROL_SET_BIT(0),
    TaskControl_IMU_DataBlunt = TASKCONTROL_SET_BIT(1),
    TaskControl_IMU_OverAngularAcc = TASKCONTROL_SET_BIT(2),
    
    /* RC Section */
    TaskControl_RC_FailSafe = TASKCONTROL_SET_BIT(3),
    TaskControl_RC_LowSig = TASKCONTROL_SET_BIT(4),
    TaskControl_RC_ErrInput = TASKCONTROL_SET_BIT(5),
}TaskControl_ErrorCode_List;

typedef struct
{
    bool init_state;
    bool control_abort;
    bool angular_protect;

    uint8_t angular_warning_cnt;

    uint8_t ctl_model;

    uint8_t imu_update_error_cnt;
    uint8_t rc_update_error_cnt;

    SYSTEM_RunTime IMU_Rt;
    SYSTEM_RunTime RC_Rt;

    bool auto_control;

    float acc_scale;
    float gyr_scale;

    float acc[Axis_Sum];
    float gyr[Axis_Sum];
    float imu_tmpr;

    float acc_lst[Axis_Sum];
    float gyr_lst[Axis_Sum];

    uint32_t error_code;
    uint8_t imu_none_update_cnt;
    uint8_t over_angular_accelerate_cnt;
} TaskControl_Monitor_TypeDef;

void TaskControl_Init(void);
void TaskControl_Core(Task_Handle hdl);

#endif