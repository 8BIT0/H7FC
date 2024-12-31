#ifndef __ATT_CASECADE_PID_H
#define __ATT_CASECADE_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pid.h"
#include "Att_Control_Base.h"

#define ATT_CASECADE_PID_PARAM_SIZE     sizeof(AttCaseCadePID_Param_TypeDef)
#define TO_ATT_CASECADE_PID_PARA_PTR(x) ((AttCaseCadePID_Param_TypeDef *)x)

#pragma pack(1)
typedef struct
{
    /* attitude parameter */
    PID_Param_TypeDef Pitch_Para;
    PID_Param_TypeDef Roll_Para;

    /* angular speed parameter */
    PID_Param_TypeDef GyroX_Para;
    PID_Param_TypeDef GyroY_Para;
    PID_Param_TypeDef GyroZ_Para;
} AttCaseCadePID_Param_TypeDef;
#pragma pack()

typedef struct
{
    bool (*set)(AttCaseCadePID_Param_TypeDef para);
    AttCaseCadePID_Param_TypeDef (*cur_param)(void);
    AttCaseCadePID_Param_TypeDef (*default_param)(void);
    bool (*process)(uint32_t sys_ms, bool angular_only, AttControl_In_TypeDef exp, AttControl_In_TypeDef mea, AngControl_Out_TypeDef *ctl_out);
    void (*reset)(void);
} AttCasecadePID_TypeDef;

extern AttCasecadePID_TypeDef Att_CasecadePID_Controller;

#ifdef __cplusplus
}
#endif

#endif
