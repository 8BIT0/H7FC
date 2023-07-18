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

typedef enum
{
    Task_SensorBaro_Core = 0,
    Task_SensorBaro_Error,
}Task_SensorBaro_State;

void TaskSample_Init(uint32_t period);
void TaskSample_Core(void const *arg);

#endif
