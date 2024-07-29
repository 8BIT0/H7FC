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

    int16_t x_diff;
    int16_t y_diff;
    int16_t x_vel;
    int16_t z_vel;
} SrvFlowObj_TypeDef;

typedef struct
{
    uint32_t time_stamp;
    int16_t x_diff;
    int16_t y_diff;
    int16_t x_vel;
    int16_t z_vel;
} SrvFlowData_TypeDef;

typedef struct
{
    bool (*init)(SrvFlowObj_TypeDef *obj);
    bool (*get_data)(SrvFlowObj_TypeDef *obj, SrvFlowData_TypeDef *p_data);
} SrvFlow_TypeDef;

extern SrvFlow_TypeDef SrvFlow;

#ifdef __cplusplus
}
#endif

#endif
