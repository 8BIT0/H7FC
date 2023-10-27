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
    Task_SensorField_IMU   = UTIL_SET_BIT(0),
    Task_SensorField_MAG   = UTIL_SET_BIT(1),
    Task_SensorField_BARO  = UTIL_SET_BIT(2),
    Task_SensorField_SONAR = UTIL_SET_BIT(3),
    Task_SensorField_TOF   = UTIL_SET_BIT(4),
    Task_SensorField_GNSS  = UTIL_SET_BIT(5),
}Task_SensorMonitor_Field_List;

typedef struct
{
    uint32_t min_sample_tick;   /* unit: us   minimun timer tick value when manipulate the bus when sensor sampling */
    uint32_t max_sample_tick;   /* unit: us   maxmun  timer tick value when manipulate the bus when sensor sampling */

    uint32_t sample_cnt;
    uint32_t err_cnt;
    uint32_t period;
}Task_SensoSampleStatistic_TypeDef;

typedef struct
{
    bool enable_monitor_timer;
    uint32_t enabled_reg;
    uint8_t enable_num;
    uint32_t init_state_reg;
    
    Task_SensoSampleStatistic_TypeDef *statistic_tab;
}Task_SensorMonitor_TypeDef;

void TaskSample_Init(uint32_t period, uint32_t sensor_enable);
void TaskSample_Core(void const *arg);

#endif
