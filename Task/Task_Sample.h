#ifndef __TASK_SAMPLE_H
#define __TASK_SAMPLE_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "cmsis_os.h"
#include "imu_data.h"
#include "Srv_SensorMonitor.h"

void TaskSample_Init(uint32_t period, uint32_t sensor_enable);
void TaskSample_Core(void const *arg);

#endif
