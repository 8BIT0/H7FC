#ifndef __TASK_CONTROL_H
#define __TASK_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "scheduler.h"
#include "imu_data.h"

#define IMU_ERROR_UPDATE_MAX_COUNT 10

typedef struct
{
    bool init_state;
    bool control_abort;

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

    float lst_acc[Axis_Sum];
    float lst_gyr[Axis_Sum];
    float lst_imu_tmpr;
} TaskControl_Monitor_TypeDef;

void TaskControl_Init(void);
void TaskControl_Core(Task_Handle hdl);

#endif