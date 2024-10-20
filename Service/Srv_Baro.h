#ifndef __SRV_BARO_H
#define __SRV_BARO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "Dev_DPS310.h"
#include "Dev_BMP280.h"
#include "../algorithm/Filter_Dep/filter.h"
#include "gen_calib.h"

#define SRVBARO_SAMPLE_RATE_LIMIT SRVBARO_SAMPLE_RATE_100HZ   /* max sample rate 100Hz */
#define SRVBARO_DEFAULT_CALI_CYCLE 100

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
    SrvBaro_Error_BusType,
    SrvBaro_Error_BadType,
    SrvBaro_Error_DevInit,
    SrvBaro_Error_BadSensorObj,
    SrvBaro_Error_FilterInit,
    SrvBaro_Error_BadSamplePeriod,
    SrvBaro_Error_BusInit,
}SrvBaro_ErrorCodeList;

typedef enum
{
    Baro_Type_None = 0,
    Baro_Type_DPS310,
    Baro_Type_BMP280,
    Baro_Type_Sum,
}SrvBaro_TypeList;

typedef struct
{
    uint32_t time_stamp;
    uint8_t cyc;

    float tempra;
    float pressure;
    float pressure_alt;
    float pressure_alt_offset;

    uint8_t error_code;
    uint8_t check_sum;
}SrvBaroData_TypeDef;

typedef union
{
    uint8_t buff[sizeof(SrvBaroData_TypeDef)];
    SrvBaroData_TypeDef data;
}SrvBaro_UnionData_TypeDef;

typedef enum
{
    SrvBaro_Bus_None = 0,
    SrvBaro_Bus_IIC,
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
    uint8_t *sensor_data;
    uint8_t data_size;

    bool ready;
    SrvBaroData_TypeDef data;
    uint32_t sample_cnt;
    uint32_t sample_err_cnt;

    uint16_t calib_cycle;
    GenCalib_State_TypeList calib_state;
    float pressure_add_sum;
    float alt_offset;

    SW_Object_Handle smoothwindow_filter_hdl;
}SrvBaroObj_TypeDef;

typedef struct
{
    uint8_t (*init)(SrvBaro_TypeList sensor_type, SrvBaroBus_TypeList bus_type);
    bool (*sample)(void);
    GenCalib_State_TypeList (*set_calib)(uint16_t calib_cyc);
    GenCalib_State_TypeList (*get_calib)(void);
    bool (*get_data)(SrvBaro_UnionData_TypeDef *data);
}SrvBaro_TypeDef;

static inline const char* SrvBaro_Get_TypeStr(SrvBaro_TypeList type)
{
    switch (type)
    {
        case Baro_Type_None:    return "None";
        case Baro_Type_DPS310:  return "DPS310";
        case Baro_Type_BMP280:  return "BMP280";
        default: return "Unknow type";
    }
}

static inline const char* SrvBaro_Get_BusStr(SrvBaroBus_TypeList bus)
{
    switch (bus)
    {
        case SrvBaro_Bus_None:  return "None";
        case SrvBaro_Bus_IIC:   return "IIC";
        case SrvBaro_Bus_SPI:   return "SPI";
        default: return "Unknow type";
    }
}

extern SrvBaro_TypeDef SrvBaro;

#ifdef __cplusplus
}
#endif

#endif
