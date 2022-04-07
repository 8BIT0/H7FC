#ifndef __IMU_DATA_H
#define __IMU_DATA_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define ConvertToTrip_Reg(x) (x << 3)

#define IMU_BLUNT_SAMPLE_CNT 50
#define IMU_ZERO_SAMPLE_CNT 50
#define IMU_OVER_RANGE_CNT 50

typedef void (*cs_ctl_callback)(bool state);
typedef bool (*bus_trans_callback)(uint8_t *tx, uint8_t *rx, uint16_t len);
typedef void (*delay_callback)(uint32_t ms);
typedef uint64_t (*get_time_stamp_callback)(void);

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
