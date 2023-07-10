#ifndef __TASK_SAMPLE_H
#define __TASK_SAMPLE_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "cmsis_os.h"
#include "imu_data.h"
#include "Srv_IMUSample.h"
#include "Srv_Baro.h"

/* task state */
typedef enum
{
    Task_SensorInertial_Core = 0,
    Task_SensorInertial_Error,
} Task_SensorInertial_State;

#pragma pack(1)
typedef struct
{
    uint32_t time_stamp;

    int8_t org_tmp;
    float org_Cnv_tmp;

    float Org_Smp[Axis_Sum];
    float Org_Cnv[Axis_Sum];
} InertialData_TypeDef;

typedef struct
{
    uint32_t time_stamp;

    InertialData_TypeDef *Inertical_Sensor;
} SensorInertial_Data_TypeDef;
#pragma pack()

void TaskSample_Init(uint32_t period);
void TaskSample_Core(void const *arg);

#endif
