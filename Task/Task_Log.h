#ifndef __TASK_LOG_H
#define __TASK_LOG_H

#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stddef.h>
#include "scheduler.h"
#include "imu_data.h"

#define LOG_HEADER 0xBA
#define LOG_DATATYPE_IMU 0x00

#define LOG_HEADER_SIZE sizeof(LogData_Header_TypeDef)

typedef union
{
    struct
    {
        uint16_t IMU_Sec : 1;
        uint16_t Res_Sec : 15;
    } _sec;

    uint16_t reg_val;
} LogData_Reg_TypeDef;

#pragma pack(1)
typedef struct
{
    uint8_t header;
    uint8_t type;
    uint8_t size;
} LogData_Header_TypeDef;

typedef struct
{
    uint64_t time;
    uint8_t cyc;
    float acc_scale;
    float gyr_scale;
    uint16_t org_acc[Axis_Sum];
    uint16_t org_gyr[Axis_Sum];
    uint16_t flt_acc[Axis_Sum];
    uint16_t flt_gyr[Axis_Sum];

    uint8_t const_res[30];

    uint8_t check_sum;
}LogIMUData_TypeDef;

typedef union
{
    uint8_t buff[sizeof(LogIMUData_TypeDef)];
    LogIMUData_TypeDef data;
}LogIMUDataUnion_TypeDef;
#pragma pack()

void TaskLog_Init(void);
void TaskLog_Core(Task_Handle hdl);

#endif
