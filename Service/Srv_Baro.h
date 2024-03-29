#ifndef __SRV_BARO_H
#define __SRV_BARO_H

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

    float tempra;
    float pressure;
    float pressure_alt;
    float pressure_alt_offset;

    uint8_t error_code;
}SrvBaroData_TypeDef;

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
    uint8_t (*init)(void);
    bool (*sample)(void);
    GenCalib_State_TypeList (*set_calib)(uint16_t calib_cyc);
    GenCalib_State_TypeList (*get_calib)(void);
    bool (*get_data)(SrvBaroData_TypeDef *data);
}SrvBaro_TypeDef;

extern SrvBaro_TypeDef SrvBaro;

#endif
