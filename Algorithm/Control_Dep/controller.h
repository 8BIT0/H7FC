#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

typedef enum
{
    CtlM_PID = 0, /* PID      control modle */
    CtlM_LADRC,   /* LADRC    control modle reserved */
    CtlM_MUDE,    /* MUDE     control modle reserved */
    CtlM_All,
} ControlMode_List;

typedef struct
{
    void *p_para;
    uint16_t size;
} ControlParam_TypeDef;

/* attitude & angular speed control */
typedef struct
{
    uint32_t time_stamp;
    ControlMode_List CtlMode;
    bool angular_only;

    /* attitude control paramter */
    ControlParam_TypeDef p_para_pitch;
    ControlParam_TypeDef p_para_roll;

    /* attitude control */
    float exp_pitch;
    float exp_roll;
    float act_pitch;
    float act_roll;

    /* angular control parameter */
    ControlParam_TypeDef *p_para_gyro_x;
    ControlParam_TypeDef *p_para_gyro_y;
    ControlParam_TypeDef *p_para_gyro_z;

    /* angular speed control */
    float exp_gyro_x;
    float exp_gyro_y;
    float exp_gyro_z;
    float act_gyro_x;
    float act_gyro_y;
    float act_gyro_z;
} AttControl_DataObj_TypeDef;

/* altitude control */
typedef struct
{
    uint32_t time_stamp;
    ControlMode_List CtlMode;
    bool abs_alt_only;

    /* parameter */
    ControlParam_TypeDef p_para_alt;

    /* relative altitude control low priority */
    float exp_rel_alt;
    float rel_alt;

    /* absolute altitude control high priority */
    float exp_abs_alt;
    float abs_alt;

    float pitch;
    float roll;
} AltControl_DataObj_TypeDef;

typedef struct
{
    bool (*att_clt_init)(AttControl_DataObj_TypeDef *D_i);
    bool (*alt_clt_init)(AltControl_DataObj_TypeDef *D_i);
    
    void (*att_ctl)(AttControl_DataObj_TypeDef *D_i, float *ctl_gyr_x, float *ctl_gyr_y, float *ctl_gyr_z);
    void (*alt_ctl)(AltControl_DataObj_TypeDef *D_i, float *ctl_throttle);
} Control_TypeDef;

extern Control_TypeDef Controller;

#endif
