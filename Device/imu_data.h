#ifndef __IMU_DATA_H
#define __IMU_DATA_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define IMU_BLUNT_SAMPLE_CNT 50
#define IMU_ZERO_SAMPLE_CNT 50
#define IMU_OVER_RANGE_CNT 50

typedef enum
{
    Axis_X = 0,
    Axis_Y,
    Axis_Z,
    Axis_Sum,
} Axis_List;

#pragma pack(1)
typedef struct
{
    uint64_t time_stamp;

    int16_t temp_int;
    int16_t gyr_int[Axis_Sum];
    int16_t acc_int[Axis_Sum];

    float temp_flt;
    double gyr_dou[Axis_Sum];
    double acc_dou[Axis_Sum];
} IMUData_TypeDef;
#pragma pack()

#endif
