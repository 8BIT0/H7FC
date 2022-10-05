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
    struct
    {
        uint8_t Pri_State : 4;
        uint8_t Sec_State : 4;
    } sec;

    uint8_t val;
} SrvMpu_Reg_TypeDef;

typedef enum
{
    SrvIMU_PriModule = 1,
    SrvIMU_SecModule,
} SrvIMU_Module_Type;

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
    SrvIMU_AllModule_Init_Error,
} SrvIMU_ErrorCode_List;

#pragma pack(1)
typedef struct
{
    SYSTEM_RunTime time_stamp;
    uint64_t cycle_cnt;

    float tempera;
    float gyr[Axis_Sum];
    float acc[Axis_Sum];
} SrvIMU_Data_TypeDef;
#pragma pack()

typedef struct
{
    SrvIMU_ErrorCode_List (*init)(void);
    bool (*sample)(void);
    SrvIMU_Data_TypeDef (*get_data)(SrvIMU_Module_Type type);
    void (*error_proc)(void);
} SrvIMU_TypeDef;

extern SrvIMU_TypeDef SrvIMU;

#endif
