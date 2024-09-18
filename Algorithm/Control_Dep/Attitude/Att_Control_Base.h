#ifndef __ATT_CONTROL_BASE_H
#define __ATT_CONTROL_BASE_H

/* expective attitude and yaw angular speed */
typedef struct
{
    float exp_pitch;
    float exp_roll;
    float exp_gyro_z;
} AttControl_ExpIn_TypeDef;

/* expectice angular speed */
typedef struct
{
    float exp_gyro_x;
    float exp_gyro_y;
    float exp_gyro_z;
} AngControl_ExpIn_TypeDef;

/* angular speed control value out */
typedef struct
{
    float out_gyro_x;   /* range 0 ~ 1.0f */
    float out_gyro_y;   /* range 0 ~ 1.0f */
    float out_gyro_z;   /* range 0 ~ 1.0f */
} AngControl_Out_TypeDef;

#endif
