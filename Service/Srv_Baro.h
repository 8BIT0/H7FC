#ifndef __SRV_BARO_H
#define __SRV_BARO_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "Dev_DPS310.h"

#define SRVBARO_SAMPLE_RATE_LIMIT SRVBARO_SAMPLE_RATE_100HZ   /* max sample rate 100Hz */

#define SRVBARO_SAMPLE_RATE_100HZ 100
#define SRVBARO_SAMPLE_RATE_50HZ  50
#define SRVBARO_SAMPLE_RATE_25HZ  25
#define SRVBARO_SAMPLE_RATE_20HZ  20
#define SRVBARO_SAMPLE_RATE_10HZ  10
#define SRVBARO_SAMPLE_RATE_5HZ   5
#define SRVBARO_SAMPLE_RATE_1HZ   1

typedef enum
{
    SrvBaro_Error_None = 0,
    SrvBaro_Error_BadRate,
    SrvBaro_Error_BadType,
    SrvBaro_Error_DevInit,
    SrvBaro_Error_BadSensorObj,
    SrvBaro_Error_BadSamplePeriod,
    SrvBaro_Error_BusInit,
}SrvBaro_ErrorCodeList;

typedef enum
{
    SrvBaro_Calibarting = 0,
    SrvBaro_CalibFailed,
    SrvBaro_CalibDone,
}SrvBaro_CalibState_List;

typedef enum
{
    Baro_Type_None = 0,
    Baro_Type_DPS310,
    Baro_Type_Sum,
}SrvBaro_TypeList;

typedef struct
{
    uint32_t time_stamp;

    uint32_t org_pres_data;
    float scaled_org_pres_data;
    float scaled_flt_pres_data;

    uint32_t org_tempra_data;
    float scaled_org_tempra_data;
    float scaled_flt_tempra_data;

    uint16_t check_sum;
}SrvBaroData_TypeDef;

typedef union
{
    uint16_t buff[sizeof(SrvBaroData_TypeDef)];
    SrvBaroData_TypeDef data;
}SrvBaro_UnionData_TypeDef;

typedef enum
{
    SrvBaro_Bus_IIC = 0,
    SrvBaro_Bus_SPI,
    SrvBaro_BusType_Sum,
}SrvBaroBus_TypeList;

typedef struct
{
    SrvBaroBus_TypeList type;
    bool init;
    void *bus_api;
    void *bus_obj;
}SrvBaroBusObj_TypeDef;

typedef struct
{
    SrvBaro_TypeList type;
    uint16_t sample_rate;   /* unit: Hz */
    uint16_t sample_period; /* unit: Ms */
    void *sensor_obj;
    void *sensor_api;
    uint8_t init_err;

    bool ready;
    SrvBaroData_TypeDef data;
    uint32_t sample_cnt;
    uint32_t sample_err_cnt;
}SrvBaroObj_TypeDef;

typedef struct
{
    uint8_t (*init)(void);
    bool (*sample)(void);
    SrvBaro_CalibState_List (*calib)(uint16_t calib_cyc, float meter);
    SrvBaro_UnionData_TypeDef (*get_data)(void);
}SrvBaro_TypeDef;

extern SrvBaro_TypeDef SrvBaro;

#endif
