#include "Att_Casecade_PID.h"

/* Casecade PID in process paramter */
typedef struct
{
    bool init;

    /* in progress parameter */
    PIDObj_TypeDef pitch;
    PIDObj_TypeDef roll;

    PIDObj_TypeDef g_x;
    PIDObj_TypeDef g_y;
    PIDObj_TypeDef g_z;
} ProcessParam_TypeDef;

/* internal vriable */
static ProcessParam_TypeDef ProcessPara = {
    .init = false;
};

/* external function */
static bool Att_CheckParam_Validation(AttCaseCadePID_Param_TypeDef para);
static bool Att_Casecade_PID(bool angular_only, AttControl_ExpIn_TypeDef exp_att, AngControl_ExpIn_TypeDef exp_ang, AngControl_Out_TypeDef ctl_out);

AttCasecadePID_TypeDef Att_CasecadePID_Controller = {
    .init = Att_CheckParam_Validation,
    .process = Att_Casecade_PID,
};

static bool Att_CheckParam_Validation(AttCaseCadePID_Param_TypeDef para)
{
    if (!ProcessPara.init)
    {
        memset();
        ProcessPara.init = true;
    }

    return false;
}

static bool Att_Casecade_PID(bool angular_only, AttControl_ExpIn_TypeDef exp_att, AngControl_ExpIn_TypeDef exp_ang, AngControl_Out_TypeDef ctl_out)
{
    return false;
}

