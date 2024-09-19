#include "Att_Casecade_PID.h"

#define PARA_AMPLIFICATE(x)     (uint16_t)(x * 1000) 
#define ATTITUDE_CONTROL_RATE   1
#define ATTITUDE_INTEGRAL_RANGE 20

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
    .init = false,
};

/* external function */
static bool Att_CheckParam_Validation(AttCaseCadePID_Param_TypeDef para);
static bool Att_Casecade_PID(bool angular_only, AttControl_ExpIn_TypeDef exp_att, AngControl_ExpIn_TypeDef exp_ang, AngControl_Out_TypeDef *ctl_out);

AttCasecadePID_TypeDef Att_CasecadePID_Controller = {
    .init = Att_CheckParam_Validation,
    .process = Att_Casecade_PID,
};

static bool Att_CheckParam_Validation(AttCaseCadePID_Param_TypeDef para)
{
    if (!ProcessPara.init)
    {
        memset(&ProcessPara, 0, sizeof(ProcessParam_TypeDef));
        ProcessPara.init = true;
    }

    /* attitude pid pitch parameter set */
    /* invalid Pitch P gain input */
    if (PARA_AMPLIFICATE(para.Pitch_Para.gP) == 0)
        return false;

    ProcessPara.pitch.gP = para.Pitch_Para.gP;
    ProcessPara.pitch.gI = para.Pitch_Para.gI;
    ProcessPara.pitch.gD = para.Pitch_Para.gD;
    ProcessPara.pitch.gI_Max = ATTITUDE_INTEGRAL_RANGE;
    ProcessPara.pitch.gI_Min = -ATTITUDE_INTEGRAL_RANGE;
    ProcessPara.pitch.diff_max = para.Pitch_Para.base_diff;
    ProcessPara.pitch.diff_min = -para.Pitch_Para.base_diff;

    /* attitude pid roll parameter set */
    /* invalid Roll P gain input */
    if (PARA_AMPLIFICATE(para.Roll_Para.gP) == 0)
        return false;

    ProcessPara.roll.gP = para.Roll_Para.gP;
    ProcessPara.roll.gI = para.Roll_Para.gI;
    ProcessPara.roll.gD = para.Roll_Para.gD;
    ProcessPara.roll.gI_Max = ATTITUDE_INTEGRAL_RANGE;
    ProcessPara.roll.gI_Min = -ATTITUDE_INTEGRAL_RANGE;
    ProcessPara.roll.diff_max = para.Roll_Para.base_diff;
    ProcessPara.roll.diff_min = -para.Roll_Para.base_diff;

    /* angular axis X parameter set */
    if (PARA_AMPLIFICATE(para.GyroX_Para.gP) == 0)
        return false;
    
    /* angular axis Y parameter set */
    if (PARA_AMPLIFICATE(para.GyroY_Para.gP) == 0)
        return false;
    
    /* angular axis Z parameter set */
    if (PARA_AMPLIFICATE(para.GyroZ_Para.gP) == 0)
        return false;

    return true;
}

static bool Att_Casecade_PID(bool angular_only, AttControl_ExpIn_TypeDef exp_att, AngControl_ExpIn_TypeDef exp_ang, AngControl_Out_TypeDef *ctl_out)
{
    bool state = false;

    if (ctl_out == NULL)
        return false;

    if (!angular_only)
    {
        /* attitude loop */
        /* Pitch PID update */
        // state = PID_Update();

        /* Roll PID update */
        // state &= PID_Update();
    }

    /* angular speed loop */
    /* Gyro X PID update */
    // state &= PID_Update();

    /* Gyro Y PID update */
    // state &= PID_Update();

    /* Gyro Z PID update */
    // state &= PID_Update();

    return state;
}

