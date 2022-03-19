#ifndef __TASK_SENSORINERTIAL_H
#define __TASK_SENSORINERTIAL_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "scheduler.h"
#include "runtime.h"

/* Axis definition */
typedef enum
{
    Axis_X = 0,
    Axis_Y,
    Axis_Z,
    Axis_Sum,
} Axis_List;

/* task state */
typedef enum
{
    Task_SensorInertial_Init = 0,
    Task_SensorInertial_Core,
    Task_SensorInertial_Error,
} Task_SensorInertial_State;

#pragma pack(1)
typedef struct
{
    SYSTEM_RunTime time_stamp;

    int8_t org_tmp;
    float org_Cnv_tmp;

    int16_t Org_Smp[Axis_Sum];
    double Org_Cnv[Axis_Sum];
} InertialData_TypeDef;

typedef struct
{
    SYSTEM_RunTime time_stamp;

    InertialData_TypeDef *Inertical_Sensor;
} SensorInertial_Data_TypeDef;
#pragma pack()

void TaskInertical_Core(Task_Handle hdl);

#endif
