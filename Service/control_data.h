#ifndef CONTROL_DATA_H
#define CONTROL_DATA_H

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
    ControlData_Src_RC = 0, /* RC remote */
    ControlData_Src_OPC,    /* On Plane Computer */
}ControlData_Source_List;

typedef enum
{
    ControlData_Type_Channel = 0,
    ControlData_Type_Attitude,
    ControlData_Type_AngularSpeed,
}ControlData_Type_List;

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
        uint16_t osd_tune : 1;
        uint16_t res : 12;
    }bit;
    
    uint16_t val;
}AUX_Control_Reg_TypeDef;

typedef struct
{
    /* channel type data section */
    uint32_t updata_time_stamp;
    uint8_t channel_cum;
    uint16_t all_ch[32];
    uint8_t gimbal_map_list[Gimbal_Sum];
    uint16_t gimbal[Gimbal_Sum];
    uint8_t gimbal_percent[Gimbal_Sum];
    
    uint32_t exp_att_time_stamp;
    float exp_att_pitch;
    float exp_att_roll;

    uint32_t exp_angularspeed_time_stamp;
    float exp_gyr_x;
    float exp_gyr_y;
    float exp_gyr_z;

    ControlData_Source_List sig_source;
    ControlData_Type_List sig_type;
    Control_Mode_List control_mode;
    
    AUX_Control_Reg_TypeDef aux;
    
    bool osd_tune_enable;
    bool arm_state;
    bool fail_safe;
}ControlData_TypeDef;

#endif

