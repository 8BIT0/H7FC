#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "Att_Casecade_PID.h"

typedef enum
{
    CtlT_Attitude = 0,
    CtlT_Altitude,
} ControlTarget_List;

typedef enum
{
    CtlM_PID = 0, /* PID      control modle */
    CtlM_LADRC,   /* LADRC    control modle reserved */
    CtlM_MUDE,    /* MUDE     control modle reserved */
    CtlM_All,
} ControlMode_List;

typedef struct
{
    bool (*att_ctl_init)(ControlMode_List mode);
    bool (*alt_ctl_init)(ControlMode_List mode);

    bool (*att_param_set)(bool ARM, ControlTarget_List target, ControlMode_List mode, uint8_t *p_param, uint16_t size);

    bool (*att_ctl)(ControlMode_List mode, uint32_t sys_ms, bool angular_only, AttControl_In_TypeDef exp, AttControl_In_TypeDef mea, AngControl_Out_TypeDef *out);
    // void (*alt_ctl)();

    void (*reset_att_processing)(ControlMode_List mode);
} Control_TypeDef;

extern Control_TypeDef Controller;

#ifdef __cplusplus
}
#endif

#endif
