#ifndef __SRV_IMUSAMPLE_H
#define __SRV_IMUSAMPLE_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "runtime.h"
#include "imu_data.h"

#define MPU_RANGE_MAX_THRESHOLD 1.2f
#define MPU_RANGE_MIN_THRESHOLD 0.9f
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

typedef enum
{
    SrvIMU_Sample_NoError = 0,
    SrvIMU_Sample_Module_UnReady,
    SrvIMU_Sample_Data_Acc_Blunt,
    SrvIMU_Sample_Data_Gyr_Blunt,
    SrvIMU_Sample_Data_Acc_OverRange,
    SrvIMU_Sample_Data_Gyr_OverRange,
} SrvIMU_SampleErrorCode_List;

#pragma pack(1)
typedef struct
{
    SYSTEM_RunTime time_stamp;
    uint64_t cycle_cnt;

    float tempera;

    float flt_gyr[Axis_Sum];
    float flt_acc[Axis_Sum];

    float org_gyr[Axis_Sum];
    float org_acc[Axis_Sum];

    SrvIMU_SampleErrorCode_List error_code;
    float acc_scale;
    float gyr_scale;
    uint16_t chk_sum;
} SrvIMU_Data_TypeDef;

typedef union
{
    uint8_t buff[sizeof(SrvIMU_Data_TypeDef)];
    SrvIMU_Data_TypeDef data;
} SrvIMU_UnionData_TypeDef;
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