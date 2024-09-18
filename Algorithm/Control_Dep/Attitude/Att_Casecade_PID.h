#ifndef __ATT_CASECADE_PID_H
#define __ATT_CASECADE_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pid.h"
#include "Att_Control_Base.h"

#define ATT_CASECADE_PID_PARAM_SIZE sizeof(AttCaseCadePID_Param_TypeDef)

typedef struct
{

    /* attitude parameter */


    /* angular speed parameter */


} AttCaseCadePID_Param_TypeDef;

typedef struct
{
    bool (*init)(AttCaseCadePID_Param_TypeDef para);
    bool (*process)(bool angular_only, AttControl_ExpIn_TypeDef exp_att, AngControl_ExpIn_TypeDef exp_ang, AngControl_Out_TypeDef ctl_out);
} AttCasecadePID_TypeDef;

extern AttCasecadePID_TypeDef Att_CasecadePID_Controller;

#ifdef __cplusplus
}
#endif

#endif
