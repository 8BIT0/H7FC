#ifndef __SRV_SENSORMONITOR_H
#define __SRV_SENSORMONITOR_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "util.h"
#include "Srv_IMUSample.h"
#include "Srv_Baro.h"

typedef enum
{
    SrvSensorMonitor_Type_IMU = 0,
    SrvSensorMonitor_Type_MAG,
    SrvSensorMonitor_Type_BARO,
    SrvSensorMonitor_Type_TOF,
    SrvSensorMonitor_Type_GNSS,
    SrvSensotMonitor_Type_SUM,
}SrvSensorMonitor_Type_List;

typedef enum
{
    SrvSensorMonitor_SampleFreq_1KHz = 0,
    SrvSensorMonitor_SampleFreq_500Hz,
    SrvSensorMonitor_SampleFreq_250Hz,
    SrvSensorMonitor_SampleFreq_200Hz,
    SrvSensorMonitor_SampleFreq_100Hz,
    SrvSensorMonitor_SampleFreq_50Hz,
    SrvSensorMonitor_SampleFreq_20Hz,
    SrvSensorMonitor_SampleFreq_10Hz,
    SrvSensorMonitor_SampleFreq_5Hz,
    SrvSensorMonitor_SampleFreq_1Hz,
}SrvSensorMonitor_SampleFreq_List;

typedef union
{
    uint32_t val;
    struct
    {
        uint32_t imu  : 1;
        uint32_t mag  : 1;
        uint32_t baro : 1;
        uint32_t tof  : 1;
        uint32_t gnss : 1;

        uint32_t res  : 27;
    }bit;
}SrvSensorMonitor_GenReg_TypeDef;

typedef union
{
    uint32_t val;
    struct
    {
        uint32_t imu  : 4;
        uint32_t mag  : 4;
        uint32_t baro : 4;
        uint32_t tof  : 4;
        uint32_t gnss : 4;

        uint32_t res  : 12;
    }bit;
}SrvSensorMonitor_SampleFreqReg_TypeDef;

typedef struct
{
    uint32_t start_time;
    uint32_t nxt_sample_time;
    uint32_t max_sampling_overhead; /* unit: us */
    uint32_t min_sampling_overhead; /* unit: us */

    uint32_t sample_cnt;
    uint32_t err_cnt;
    uint32_t detect_period;
    uint32_t set_period;
}SrvSensorMonitor_Statistic_TypeDef;

/* bit field on init_state_reg set 1 represent error triggerd on */
typedef struct
{
    SrvSensorMonitor_GenReg_TypeDef enabled_reg;        /* pipe this vriable to datahub after srv_sensormonitor init */
    SrvSensorMonitor_GenReg_TypeDef init_state_reg;     /* pipe thie vriable to datahub after srv_sensormonitor init */
    SrvSensorMonitor_SampleFreqReg_TypeDef freq_reg;
    
    SrvSensorMonitor_Statistic_TypeDef *statistic_list;
 }SrvSensorMonitorObj_TypeDef;

typedef struct
{
    bool (*init)(SrvSensorMonitorObj_TypeDef *obj);
    bool (*sample_ctl)(SrvSensorMonitorObj_TypeDef *obj);
    SrvIMU_UnionData_TypeDef (*get_imu_data)(SrvSensorMonitorObj_TypeDef *obj);
}SrvSensorMonitor_TypeDef;

extern SrvSensorMonitor_TypeDef SrvSensorMonitor;
#endif
