#ifndef __ATT_CASECADE_PID_H
#define __ATT_CASECADE_PID_H

#include "pid.h"
#include "Att_Control_Base.h"

#define ATT_CASECADE_PID_PARAM_SIZE sizeof(AttCaseCadePID_Param_TypeDef)

typedef struct
{

} AttCaseCadePID_Param_TypeDef;

typedef struct
{
    bool (*check_param_validation)(AttCaseCadePID_Param_TypeDef para);
    bool (*control)(AttCaseCadePID_Param_TypeDef *para, bool angular_only, AttControl_ExpIn_TypeDef exp_att, AngControl_ExpIn_TypeDef exp_ang, AngControl_Out_TypeDef ctl_out);
} AttCasecadePID_TypeDef;

extern AttCasecadePID_TypeDef Att_CasecadePID_Controller;

#endif
