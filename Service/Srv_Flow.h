#ifndef __SRV_FLOW_H
#define __SRV_FLOW_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "Srv_OsCommon.h"
#include "pos_data.h"
#include "Dev_Flow_3901_L0X.h"

typedef enum
{
    SrvFlow_None = 0,
    SrvFlow_MATEK_3901_L0X,
} SrvFlow_SensorType_List;

typedef struct
{
    SrvFlow_SensorType_List type;
    bool init;

    uint32_t time_stamp;
    PosData_TypeDef pos;
    PosVelData_TypeDef vel;
} SrvFlowObj_TypeDef;

typedef struct
{
    bool (*init)(SrvFlowObj_TypeDef *obj);
    bool (*get_pos)(SrvFlowObj_TypeDef *obj, PosData_TypeDef *pos);
    bool (*get_vel)(SrvFlowObj_TypeDef *obj, PosVelData_TypeDef *vel);
} SrvFlow_TypeDef;

extern SrvFlow_TypeDef SrvFlow;

#ifdef __cplusplus
}
#endif

#endif
