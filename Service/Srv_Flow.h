#ifndef __SRV_FLOW_H
#define __SRV_FLOW_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "Srv_OsCommon.h"

typedef enum
{
    Flow_None = 0,
    Flow_3901U,
} SrvFlow_SensorType_List;

typedef struct
{
    SrvFlow_SensorType_List type;
    bool init;

    void *api;
    void *obj;

    uint32_t time_stamp;
    uint32_t thread_freq;

    float flt_x_i;
    float flt_y_i;
    float att_x_bias;
    float att_y_bias;

    uint32_t lst_time_stamp;
    float lst_out_x;
    float lst_out_y;

    float out_x;
    float out_y;
    float x_v;
    float y_v;
} SrvFlowObj_TypeDef;

typedef struct
{
    uint32_t time_stamp;
    int16_t x_i;            /* distance integrate on x axis unit: cm */
    int16_t y_i;            /* distance integrate on y axis unit: cm */
    int16_t x_v;
    int16_t y_v;
} SrvFlowData_TypeDef;

typedef struct
{
    bool (*init)(SrvFlow_SensorType_List type);
    bool (*get_data)(SrvFlowData_TypeDef *p_data);
} SrvFlow_TypeDef;

extern SrvFlow_TypeDef SrvFlow;

#ifdef __cplusplus
}
#endif

#endif
