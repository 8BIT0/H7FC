#include "Att_Casecade_PID.h"

#define DYNAMIC_TUNE_THROTTLE_POINT 0.4f

#define PARA_AMPLIFICATE(x)         (uint16_t)(x * 1000) 
#define ATTITUDE_INTEGRAL_RANGE     20

#define ATT_DTRIM_RC_F_CUT          20
#define ANG_DTRIM_RC_F_CUT          100

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

/* internal function */
static bool Att_PID_Param_Set(AttCaseCadePID_Param_TypeDef para);

/* external function */
static bool Att_CheckParam_Validation(AttCaseCadePID_Param_TypeDef para);
static bool Att_Casecade_PID(uint32_t sys_ms, bool angular_only, AttControl_In_TypeDef exp, AttControl_In_TypeDef mea, AngControl_Out_TypeDef *ctl_out);
static void Att_Casecade_PID_Reset(void);
static AttCaseCadePID_Param_TypeDef Att_Casecade_PID_DefaultPara(void);
static AttCaseCadePID_Param_TypeDef Att_CaseCade_PID_InUse_Param(void);

AttCasecadePID_TypeDef Att_CasecadePID_Controller = {
    .set = Att_CheckParam_Validation,
    .process = Att_Casecade_PID,
    .cur_param = Att_CaseCade_PID_InUse_Param,
    .default_param = Att_Casecade_PID_DefaultPara,
    .reset = Att_Casecade_PID_Reset,
};

static AttCaseCadePID_Param_TypeDef Att_CaseCade_PID_InUse_Param(void)
{
    AttCaseCadePID_Param_TypeDef in_use_para;
    memset(&in_use_para, 0, sizeof(AttCaseCadePID_Param_TypeDef));

    in_use_para.Pitch_Para.gP = ProcessPara.pitch.gP;
    in_use_para.Pitch_Para.gI = ProcessPara.pitch.gI;
    in_use_para.Pitch_Para.gD = ProcessPara.pitch.gD;

    in_use_para.Roll_Para.gP  = ProcessPara.roll.gP;
    in_use_para.Roll_Para.gI  = ProcessPara.roll.gI;
    in_use_para.Roll_Para.gD  = ProcessPara.roll.gD;
     
    in_use_para.GyroX_Para.gP = ProcessPara.g_x.gP;
    in_use_para.GyroX_Para.gI = ProcessPara.g_x.gI;
    in_use_para.GyroX_Para.gD = ProcessPara.g_x.gD;

    in_use_para.GyroY_Para.gP = ProcessPara.g_y.gP;
    in_use_para.GyroY_Para.gI = ProcessPara.g_y.gI;
    in_use_para.GyroY_Para.gD = ProcessPara.g_y.gD;
    
    in_use_para.GyroZ_Para.gP = ProcessPara.g_z.gP;
    in_use_para.GyroZ_Para.gI = ProcessPara.g_z.gI;
    in_use_para.GyroZ_Para.gD = ProcessPara.g_z.gD;

    return in_use_para;
}

static AttCaseCadePID_Param_TypeDef Att_Casecade_PID_DefaultPara(void)
{
    AttCaseCadePID_Param_TypeDef para;

    memset(&para, 0, sizeof(AttCaseCadePID_Param_TypeDef));

    para.Pitch_Para.gP          = 1.2;
    para.Pitch_Para.gI          = 0.2;
    para.Pitch_Para.gI_Max      = 10;
    para.Pitch_Para.gI_Min      = -10;
    para.Pitch_Para.gD          = 0.5;

    para.Roll_Para.gP           = 1.2;
    para.Roll_Para.gI           = 0.2;
    para.Roll_Para.gI_Max       = 10;
    para.Roll_Para.gI_Min       = -10;
    para.Roll_Para.gD           = 0.5;

    para.GyroX_Para.gP          = 1.4;
    para.GyroX_Para.gI          = 0.006;
    para.GyroX_Para.gI_Max      = 100;
    para.GyroX_Para.gI_Min      = -100;
    para.GyroX_Para.gD          = 1.8;

    para.GyroY_Para.gP          = 1.4;
    para.GyroY_Para.gI          = 0.006;
    para.GyroY_Para.gI_Max      = 100;
    para.GyroY_Para.gI_Min      = -100;
    para.GyroY_Para.gD          = 1.8;

    para.GyroZ_Para.gP          = 1.3;
    para.GyroZ_Para.gI          = 0.008;
    para.GyroZ_Para.gI_Max      = 50;
    para.GyroZ_Para.gI_Min      = -50;
    para.GyroZ_Para.gD          = 1.2;

    return para;
}

static bool Att_PID_Param_Set(AttCaseCadePID_Param_TypeDef para)
{
    /* attitude pid pitch parameter set */
    /* invalid Pitch P gain input */
    if (PARA_AMPLIFICATE(para.Pitch_Para.gP) == 0)
    {
        /* use default parameter */
        return false;
    }

    ProcessPara.pitch.gP = para.Pitch_Para.gP;
    ProcessPara.pitch.gI = para.Pitch_Para.gI;
    ProcessPara.pitch.gD = para.Pitch_Para.gD;
    ProcessPara.pitch.gI_Max = ATTITUDE_INTEGRAL_RANGE;
    ProcessPara.pitch.gI_Min = -ATTITUDE_INTEGRAL_RANGE;

    /* attitude pid roll parameter set */
    /* invalid Roll P gain input */
    if (PARA_AMPLIFICATE(para.Roll_Para.gP) == 0)
    {
        /* use default parameter */
        return false;
    }

    ProcessPara.roll.gP = para.Roll_Para.gP;
    ProcessPara.roll.gI = para.Roll_Para.gI;
    ProcessPara.roll.gD = para.Roll_Para.gD;
    ProcessPara.roll.gI_Max = ATTITUDE_INTEGRAL_RANGE;
    ProcessPara.roll.gI_Min = -ATTITUDE_INTEGRAL_RANGE;

    /* angular axis X parameter set */
    if (PARA_AMPLIFICATE(para.GyroX_Para.gP) == 0)
    {
        /* use default parameter */
        return false;
    }

    ProcessPara.g_x.gP = para.GyroX_Para.gP;
    ProcessPara.g_x.gI = para.GyroX_Para.gI;
    ProcessPara.g_x.gD = para.GyroX_Para.gD;
    ProcessPara.g_x.gI_Limit = false;
    ProcessPara.g_x.gI_Min = 50;
    ProcessPara.g_x.gI_Max = -50;
    
    /* angular axis Y parameter set */
    if (PARA_AMPLIFICATE(para.GyroY_Para.gP) == 0)
    {
        /* use default parameter */
        return false;
    }

    ProcessPara.g_y.gP = para.GyroY_Para.gP;
    ProcessPara.g_y.gI = para.GyroY_Para.gI;
    ProcessPara.g_y.gI_Limit = false;
    ProcessPara.g_y.gI_Min = 50;
    ProcessPara.g_y.gI_Max = -50;
    ProcessPara.g_y.gD = para.GyroY_Para.gD;
                   
    /* angular axis Z parameter set */
    if (PARA_AMPLIFICATE(para.GyroZ_Para.gP) == 0)
    {
        /* use default parameter */
        return false;
    }

    ProcessPara.g_z.gP = para.GyroZ_Para.gP;
    ProcessPara.g_z.gI = para.GyroZ_Para.gI;
    ProcessPara.g_z.gI_Limit = false;
    ProcessPara.g_z.gI_Min = 50;
    ProcessPara.g_z.gI_Max = -50;
    ProcessPara.g_z.gD = para.GyroZ_Para.gD;

    return true;
}

static bool Att_CheckParam_Validation(AttCaseCadePID_Param_TypeDef para)
{
    RC_Filter_Param_TypeDef RCParam_Tmp;
    bool state = true;

    if (!ProcessPara.init)
        memset(&ProcessPara, 0, sizeof(ProcessParam_TypeDef));

    if (!Att_PID_Param_Set(para))
    {
        para = Att_Casecade_PID_DefaultPara(); 
        state = false;
    }

    RCParam_Tmp.f_cut = ATT_DTRIM_RC_F_CUT;
    PID_Init(&ProcessPara.pitch, RCParam_Tmp);
    PID_Init(&ProcessPara.roll,  RCParam_Tmp);

    RCParam_Tmp.f_cut = ANG_DTRIM_RC_F_CUT;
    PID_Init(&ProcessPara.g_x, RCParam_Tmp);
    PID_Init(&ProcessPara.g_y, RCParam_Tmp);
    PID_Init(&ProcessPara.g_z, RCParam_Tmp);
    
    ProcessPara.init = true;
    return state;
}

static bool Att_Casecade_PID(uint32_t sys_ms, bool angular_only, AttControl_In_TypeDef exp, AttControl_In_TypeDef mea, AngControl_Out_TypeDef *ctl_out)
{
    float dynamic_tune_rate = 1.0f;
    bool dynamic_tune_en = false;

    if (ctl_out == NULL)
        return false;

    if (!angular_only)
    {
        /* attitude loop */
        /* dynamic trim is unavilable in this loop */
        /* Pitch PID update */
        PID_Update(&ProcessPara.pitch, sys_ms, mea.pitch, exp.pitch);
        exp.gyro_y = ProcessPara.pitch.fout;

        /* Roll PID update */
        PID_Update(&ProcessPara.roll, sys_ms, mea.roll, exp.roll);
        exp.gyro_x = ProcessPara.roll.fout;
    }
    else
    {
        ProcessPara.pitch.lst_diff = 0.0f;
        ProcessPara.pitch.Integral = 0.0f;

        ProcessPara.roll.lst_diff = 0.0f;
        ProcessPara.roll.Integral = 0.0f;
    }

    if (exp.throttle_percent >= DYNAMIC_TUNE_THROTTLE_POINT)
    {
        dynamic_tune_en = true;

        /* comput dynamic tune rate */
        dynamic_tune_rate -= (exp.throttle_percent - DYNAMIC_TUNE_THROTTLE_POINT);
    }

    PID_P_DynamicTrim(&ProcessPara.g_x, dynamic_tune_en, dynamic_tune_rate);
    PID_P_DynamicTrim(&ProcessPara.g_y, dynamic_tune_en, dynamic_tune_rate);
    PID_P_DynamicTrim(&ProcessPara.g_z, dynamic_tune_en, dynamic_tune_rate);

    PID_D_DynamicTrim(&ProcessPara.g_x, dynamic_tune_en, dynamic_tune_rate);
    PID_D_DynamicTrim(&ProcessPara.g_y, dynamic_tune_en, dynamic_tune_rate);
    PID_D_DynamicTrim(&ProcessPara.g_z, dynamic_tune_en, dynamic_tune_rate);

    /* angular speed loop */
    /* Gyro X PID update */
    PID_Update(&ProcessPara.g_x, sys_ms, mea.gyro_x, exp.gyro_x);

    /* Gyro Y PID update */
    PID_Update(&ProcessPara.g_y, sys_ms, mea.gyro_y, exp.gyro_y);

    /* Gyro Z PID update */
    PID_Update(&ProcessPara.g_z, sys_ms, mea.gyro_z, exp.gyro_z);

    ctl_out->gyro_x = ProcessPara.g_x.fout;
    ctl_out->gyro_y = ProcessPara.g_y.fout;
    ctl_out->gyro_z = ProcessPara.g_z.fout;

    return true;
}

static void Att_Casecade_PID_Reset(void)
{
    PID_Reset_ProcessVal(&ProcessPara.pitch);
    PID_Reset_ProcessVal(&ProcessPara.roll);
    PID_Reset_ProcessVal(&ProcessPara.g_x);
    PID_Reset_ProcessVal(&ProcessPara.g_y);
    PID_Reset_ProcessVal(&ProcessPara.g_z);
}
