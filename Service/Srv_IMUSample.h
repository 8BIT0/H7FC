#ifndef __SRV_IMUSAMPLE_H
#define __SRV_IMUSAMPLE_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "Srv_OsCommon.h"
#include "imu_data.h"
#include "util.h"

#define MPU_RANGE_MAX_THRESHOLD 1.2f
#define MPU_RANGE_MIN_THRESHOLD 0.9f
#define MPU_MODULE_OPR_DURATION 50 // duration time 50ms

typedef union
{
    struct
    {
        uint8_t Pri_State : 2;
        uint8_t Sec_State : 2;
        uint8_t Fus_State : 2;
        uint8_t res       : 2;
    } sec;

    uint8_t val;
} SrvMpu_Reg_TypeDef;

typedef enum
{
    SrvIMU_PriModule = 1,
    SrvIMU_SecModule,
    SrvIMU_FusModule,
} SrvIMU_Module_Type;

typedef enum
{
    SrvIMU_PriDev_Detect_Error = -12,
    SrvIMU_SecDev_Detect_Error = -11,
    SrvIMU_SecIMU_Filter_Init_Error = -10,
    SrvIMU_PriIMU_Filter_Init_Error = -9,
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
    SrvIMU_Sample_Over_Angular_Accelerate,
} SrvIMU_SampleErrorCode_List;

typedef enum
{
    SrvIMU_Dev_MPU6000 = 0,
    SrvIMU_Dev_ICM20602,
    SrvIMU_Dev_ICM42688P,
    SrvIMU_Dev_ICM42605,
    SrvIMU_Dev_None,
}SrvIMU_SensorID_List;

typedef enum
{
    SrvIMU_Gyr_Calibarting,
    SrvIMU_Gyr_CalibFailed,
    SrvIMU_Gyr_CalibDone,
}SrvIMU_GyroCalib_State_List;

typedef enum
{
    SrvIMU_Priori_Pri = UTIL_SET_BIT(0),                        /* primary IMU sample Only */
    SrvIMU_Priori_Sec = UTIL_SET_BIT(1),                        /* secondary IMU sample Only */
    SrvIMU_Both_Sample = SrvIMU_Priori_Pri | SrvIMU_Priori_Sec, /* both primary and secondary IMU sample */
}SrvIMU_SampleMode_List;

#pragma pack(1)
typedef struct
{
    uint32_t time_stamp;
    uint32_t cycle_cnt;

    float tempera;

    float flt_gyr[Axis_Sum];
    float flt_acc[Axis_Sum];

    float org_gyr[Axis_Sum];
    float org_acc[Axis_Sum];

    float max_gyr_angular_diff;

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
    bool (*sample)(SrvIMU_SampleMode_List mode);
    SrvIMU_Data_TypeDef (*get_data)(SrvIMU_Module_Type type);
    float (*get_max_angular_speed_diff)(void);
    void (*error_proc)(void);
    SrvIMU_GyroCalib_State_List (*calib)(const uint32_t calib_cycle, float *pri_gyr, float *sec_gyr);
} SrvIMU_TypeDef;

extern SrvIMU_TypeDef SrvIMU;

#endif
