#ifndef CONTROL_DATA_H
#define CONTROL_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef enum
{
    Channel_1 = 0,
    Channel_2,
    Channel_3,
    Channel_4,
    Channel_5,
    Channel_6,
    Channel_7,
    Channel_8,
    Channel_9,
    Channel_10,
    Channel_11,
    Channel_12,
    Channel_13,
    Channel_14,
    Channel_15,
    Channel_16,
    Channel_Max,
}Control_Channel_List;

typedef enum
{
    Gimbal_Throttle = 0,
    Gimbal_Pitch,
    Gimbal_Roll,
    Gimbal_Yaw,
    Gimbal_Sum,
}Control_GimbalMapDef_List;

typedef enum
{
    Attitude_Control = 0,
    AngularSpeed_Control,
    AngularSpeed_AngleLimit_Control,
}Control_Mode_List;

typedef union
{
    struct
    {
        uint16_t hover_pos_hold : 1;
        uint16_t buzzer : 1;
        uint16_t flip_over : 1;
        uint16_t calib : 1;
        uint16_t blackbox : 1;
        uint16_t res : 12;
    }bit;
    
    uint16_t val;
}AUX_Control_Reg_TypeDef;

/* parse from receiver */
typedef struct
{
    /* channel type data section */
    uint32_t update_time_stamp;
    uint8_t channel_sum;
    uint16_t all_ch[32];
    int16_t gimbal[Gimbal_Sum];
    uint8_t throttle_percent;
    uint8_t pitch_percent;
    uint8_t roll_percent;
    uint8_t yaw_percent;

    int16_t gimbal_max;
    int16_t gimbal_mid;
    int16_t gimbal_min;

    Control_Mode_List control_mode;
    AUX_Control_Reg_TypeDef aux;
    
    bool arm_state;
    bool fail_safe;

    uint16_t rssi;
}ControlData_TypeDef;

/* convert control data from rc receiver */
typedef struct
{
    uint32_t time_stamp;

    bool arm;
    bool failsafe;
    Control_Mode_List control_mode;

    float pitch;
    float roll;

    float gyr_x;
    float gyr_y;
    float gyr_z;
} ExpControlData_TypeDef;

#ifdef __cplusplus
}
#endif

#endif

