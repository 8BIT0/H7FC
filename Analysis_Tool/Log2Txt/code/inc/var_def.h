#ifndef __VAR_DEF_H
#define __VAR_DEF_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h>

typedef uint64_t SYSTEM_RunTime;
#define LOG_HEADER 0xBA
#define LOG_DATATYPE_IMU 0x00

#define LOG_HEADER_SIZE sizeof(LogData_Header_TypeDef)
#define LOG_IMU_DATA_SIZE sizeof(IMU_LogUnionData_TypeDef)

typedef enum
{
    Axis_X = 0,
    Axis_Y,
    Axis_Z,
    Axis_Sum,
} AxisList_TypeDef;

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
    uint8_t header;
    uint8_t type;
    uint8_t size;
} LogData_Header_TypeDef;

typedef struct
{
    SYSTEM_RunTime time_stamp;
    uint64_t cycle_cnt;

    float tempera;
    float gyr[Axis_Sum];
    float acc[Axis_Sum];

    SrvIMU_SampleErrorCode_List error_code;
    uint16_t chk_sum;
} IMU_Data_TypeDef;

typedef union
{
    uint8_t buff[sizeof(IMU_Data_TypeDef)];
    IMU_Data_TypeDef data;
} IMU_LogUnionData_TypeDef;
#pragma pack()

#endif
