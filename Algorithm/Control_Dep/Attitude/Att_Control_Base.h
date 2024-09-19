#ifndef __ATT_CONTROL_BASE_H
#define __ATT_CONTROL_BASE_H

#ifdef __cplusplus
extern "C" {
#endif

/* attitude and angular speed */
typedef struct
{
    float pitch;
    float roll;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} AttControl_In_TypeDef;

/* angular speed control value out */
typedef struct
{
    float out_gyro_x;   /* range 0 ~ 1.0f */
    float out_gyro_y;   /* range 0 ~ 1.0f */
    float out_gyro_z;   /* range 0 ~ 1.0f */
} AngControl_Out_TypeDef;

#ifdef __cplusplus
}
#endif

#endif
