#ifndef __IMU_DATA_H
#define __IMU_DATA_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define ConvertToTrip_Reg(x) (x << 3)

/*
 *  CNT 100 in 2K Sample Frequence is about keeping 50Ms abnormality data
 */
#define IMU_BLUNT_SAMPLE_CNT 100

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
    uint64_t cycle_cnt;

    int16_t temp_int;
    int16_t gyr_int[Axis_Sum];
    int16_t acc_int[Axis_Sum];

    float temp_flt;
    float gyr_flt[Axis_Sum];
    float acc_flt[Axis_Sum];

    int16_t gyr_int_lst[Axis_Sum];
    int16_t acc_int_lst[Axis_Sum];

    uint8_t gyr_blunt_cnt[Axis_Sum];
    uint8_t acc_blunt_cnt[Axis_Sum];
} IMUData_TypeDef;
#pragma pack()

#endif
