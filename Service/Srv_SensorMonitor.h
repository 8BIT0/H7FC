#ifndef __SRV_SENSORMONITOR_H
#define __SRV_SENSORMONITOR_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "util.h"
#include "Bsp_Timer.h"
#include "Srv_IMUSample.h"
#include "Srv_Baro.h"

typedef SrvIMU_Range_TypeDef SrvSensorMonitor_IMURange_TypeDef;

#define GYRO_CALIB_CYCLE GYR_STATIC_CALIB_CYCLE
#define BARO_CALIB_CYCLE SRVBARO_DEFAULT_CALI_CYCLE

typedef enum
{
    SrvSensorMonitor_StatisticTimer_Defualt = 0,
    SrvSensorMonitor_StatisticTimer_NoError,
    SrvSensorMonitor_StatisticTimer_Error,
}SrvSensorMonitor_StatisticTimer_State_List;

typedef enum
{
    SrvSensorMonitor_Type_IMU = 0,
    SrvSensorMonitor_Type_MAG,
    SrvSensorMonitor_Type_BARO,
    SrvSensorMonitor_Type_TOF,
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

        uint32_t res  : 12;
    }bit;
}SrvSensorMonitor_SampleFreqReg_TypeDef;

typedef struct
{
    uint32_t start_time;
    uint32_t nxt_sample_time;
    uint32_t cur_sampling_overhead; /* unit: 100ns */
    uint32_t max_sampling_overhead; /* unit: 100ns */
    uint32_t min_sampling_overhead; /* unit: 100ns */
    uint32_t avg_sampling_overhead; /* unit: 100ns */

    GenCalib_State_TypeList is_calid;
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

    SrvSensorMonitor_Statistic_TypeDef *statistic_imu;
    SrvSensorMonitor_Statistic_TypeDef *statistic_mag;
    SrvSensorMonitor_Statistic_TypeDef *statistic_baro;
    SrvSensorMonitor_Statistic_TypeDef *statistic_tof;

    SrvSensorMonitor_Statistic_TypeDef *statistic_list;

    uint8_t imu_num;
    bool pri_range_get;
    bool sec_range_get;
    SrvSensorMonitor_IMURange_TypeDef PriIMU_Range;
    SrvSensorMonitor_IMURange_TypeDef SecIMU_Range;

    SrvIMU_SampleMode_List IMU_SampleMode;

    SrvIMU_UnionData_TypeDef lst_imu_data;
    SrvBaroData_TypeDef lst_baro_data;
 }SrvSensorMonitorObj_TypeDef;

typedef struct
{
    bool (*init)(SrvSensorMonitorObj_TypeDef *obj);
    bool (*sample_ctl)(SrvSensorMonitorObj_TypeDef *obj);
    bool (*get_imu_num)(SrvSensorMonitorObj_TypeDef *obj, uint8_t *num);
    bool (*get_imu_range)(SrvSensorMonitorObj_TypeDef *obj, SrvIMU_Module_Type type, SrvSensorMonitor_IMURange_TypeDef *range);
    SrvIMU_UnionData_TypeDef (*get_imu_data)(SrvSensorMonitorObj_TypeDef *obj);
    SrvBaroData_TypeDef (*get_baro_data)(SrvSensorMonitorObj_TypeDef *obj);
    GenCalib_State_TypeList (*set_calib)(SrvSensorMonitorObj_TypeDef *obj, SrvSensorMonitor_Type_List type);
    GenCalib_State_TypeList (*get_calib)(SrvSensorMonitorObj_TypeDef *obj, SrvSensorMonitor_Type_List type);
}SrvSensorMonitor_TypeDef;

extern SrvSensorMonitor_TypeDef SrvSensorMonitor;
#endif
