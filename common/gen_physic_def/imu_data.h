#ifndef __IMU_DATA_H
#define __IMU_DATA_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/*
 *  CNT 100 in 2K Sample Frequence is about keeping 50Ms abnormality data
 */
#define IMU_BLUNT_SAMPLE_CNT 100

typedef void (*cs_ctl_callback)(bool state);
typedef bool (*bus_trans_callback)(uint8_t *tx, uint8_t *rx, uint16_t len);
typedef int32_t (*delay_callback)(uint32_t ms);
typedef uint32_t (*get_time_stamp_callback)(void);

typedef enum
{
    Att_Pitch = 0,
    Att_Roll,
    Att_Ctl_Sum,
    Att_Yaw = Att_Ctl_Sum,
    Att_Heading,
} Attitude_List;

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
    uint32_t time_stamp;
    uint8_t cycle_cnt;

    int16_t temp_int;
    int16_t gyr_int[Axis_Sum];
    int16_t acc_int[Axis_Sum];

    float temp_flt;
    float gyr_flt[Axis_Sum];
    float acc_flt[Axis_Sum];

    int16_t gyr_int_lst[Axis_Sum];
    int16_t acc_int_lst[Axis_Sum];

    uint16_t gyr_blunt_cnt[Axis_Sum];
    uint16_t acc_blunt_cnt[Axis_Sum];
} IMUData_TypeDef;

typedef struct
{
    float acc_scale;
    float gyr_scale;
} IMUModuleScale_TypeDef;

typedef struct
{
    uint32_t time_stamp;
    float pitch;
    float roll;
    float yaw;
    float q0;
    float q1;
    float q2;
    float q3;
    bool flip_over;
    uint8_t err_code;
} IMUAtt_TypeDef;
#pragma pack()

typedef struct
{
    uint8_t code;
    char* function;
    uint16_t line;

    uint8_t tar_reg;
    uint8_t reg_r_val;
    uint8_t reg_t_val;
}IMU_Error_TypeDef;

inline void IMUData_SetError(IMU_Error_TypeDef *error, uint8_t code, char* func, uint16_t line, uint8_t reg, uint8_t reg_r, uint8_t reg_t)
{
    if(error)
    {
        error->code = code;
        error->function = func;
        error->line = line;

        error->tar_reg = reg;
        error->reg_r_val = reg_r;
        error->reg_t_val = reg_t;
    }
}

#endif
