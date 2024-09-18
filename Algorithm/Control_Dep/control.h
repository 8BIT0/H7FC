#ifndef __CONTROL_H
#define __CONTROL_H

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

typedef enum
{
    CtlM_PID = 0, /* PID      control modle */
    CtlM_LADRC,   /* LADRC    control modle reserved */
    CtlM_MUDE,    /* MUDE     control modle reserved */
} ControlMode_List;

/* attitude & angular speed control */
typedef struct
{
    uint32_t time_stamp;
    ControlMode_List CtlMode;
    bool angular_only;

    /* attitude control */
    float exp_pitch;
    float exp_roll;
    float act_pitch;
    float act_roll;

    /* angular speed control */
    float exp_gyro_x;
    float exp_gyro_y;
    float exp_gyro_z;
    float act_gyro_x;
    float act_gyro_y;
    float act_gyro_z;

    float gyro_x_ctl;   /* range 0 ~ 1 */
    float gyro_y_ctl;   /* range 0 ~ 1 */
    float gyro_z_ctl;   /* range 0 ~ 1 */
} AttControl_DataObj_TypeDef;

/* altitude control */
typedef struct
{
    uint32_t time_stamp;

    float rel_alt;
    float abs_alt;

    float pitch;
    float roll;
} AltControl_DataObj_TypeDef;

typedef struct
{
    void (*att_ctl)(AttControl_DataObj_TypeDef D_i);
    void (*alt_ctl)();
} Control_TypeDef;

extern Control_TypeDef Controller;

#endif
