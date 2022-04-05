#ifndef __SRVMPU_SAMPLE_H
#define __SRVMPU_SAMPLE_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "runtime.h"
#include "imu_data.h"

#define MPU_MODULE_OPR_DURATION 50 // duration time 50ms

typedef union
{
    uint8_t PriDev_Init_State : 4;
    uint8_t SecDev_Init_State : 4;
} SrvMpu_InitReg_TypeDef;

typedef enum
{
    SrvIMU_PriCSPin_Init_Error = -8,
    SrvIMU_PriExtiPin_Init_Error = -7,
    SrvIMU_PriBus_Init_Error = -6,
    SrvIMU_PriDev_Init_Error = -5,
    SrvIMU_SecCSPin_Init_Error = -4,
    SrvIMU_SecExtiPin_Init_Error = -3,
    SrvIMU_SecBus_Init_Error = -2,
    SrvIMU_SecDev_Init_Error = -1,
    SrvIMU_No_Error = 0,
    SrvIMU_Sample_Init_Error,
    SrvIMU_Sample_OverRange,
    SrvIMU_Sample_Blunt,
} SrvIMU_ErrorCode_List;

SrvIMU_ErrorCode_List SrvIMU_Init(void);
int8_t SrvIMU_GetPri_InitError(void);

#endif
