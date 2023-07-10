#ifndef __SRV_BARO_H
#define __SRV_BARO_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#define SRVBARO_SAMPLE_RATE_LIMIT 100   /* max sample rate 100Hz */

typedef enum
{
    SrvBaro_Error_None = 0,
    SrvBaro_Error_BadObj,
    SrvBaro_Error_BadRate,
    SrvBaro_Error_BadType,
    SrvBaro_Error_BadSensorObj,
    SrvBaro_Error_BadBusObj,
    SrvBaro_Error_BadBusApi,
    SrvBaro_Error_BusInit,
}SrvBaro_ErrorList;

typedef enum
{
    Baro_Type_None = 0,
    Baro_Type_DPS310,
    Baro_Type_Sum,
}SrvBaro_TypeList;

typedef struct
{
    int16_t org_data;
    float scaled_org_data;
    float scaled_flt_data;

    uint8_t err_code;
}SrvBaroData_TypeDef;

typedef struct
{
    void *bus_obj;
    bool (*bus_init)(void *bus_obj);
    bool (*bus_read)(void *bus_obj, uint8_t addr, uint8_t *p_data, uint16_t len);
    bool (*bus_write)(void *bus_obj, uint8_t addr, uint8_t *p_data, uint16_t len);
}SrvBaroBus_TypeDef;

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
    SrvBaroBus_TypeDef bus;
}SrvBaroObj_TypeDef;

typedef struct
{
    uint8_t (*init)(SrvBaroObj_TypeDef *obj, SrvBaro_TypeList type, uint16_t rate, SrvBaroBus_TypeDef bus);
    bool (*ready)(SrvBaroObj_TypeDef *obj);
    bool (*sample)(SrvBaroObj_TypeDef *obj);
    SrvBaroData_TypeDef (*get)(SrvBaroData_TypeDef *obj);
}SrvBaro_TypeDef;

extern SrvBaro_TypeDef SrvBaro;

#endif
