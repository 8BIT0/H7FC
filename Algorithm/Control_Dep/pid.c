/* Author: 8_B!T0
 */
#include "pid.h"

#define Default_gP_Max_Rate 1.0f
#define Default_gD_Max_Rate 1.0f

#define Default_gP_Min_Rate 0.4f
#define Default_gD_Min_Rate 0.4f

/* internal function */
static bool PID_P_Progress(PIDObj_TypeDef *p_PIDObj, const float diff);
static bool PID_I_Progress(PIDObj_TypeDef *p_PIDObj, const float diff);
static bool PID_D_Progress(PIDObj_TypeDef *p_PIDObj, uint32_t sys_ms, const float diff);

bool PID_Init(PIDObj_TypeDef *p_PIDObj, RC_Filter_Param_TypeDef rc_para)
{
    if (p_PIDObj == NULL)
        return false;

    p_PIDObj->in = 0.0f;
    p_PIDObj->exp = 0.0f;
    p_PIDObj->fout = 0.0f;

    p_PIDObj->P_out = 0.0f;
    p_PIDObj->I_out = 0.0f;
    p_PIDObj->D_out = 0.0f;

    p_PIDObj->gP_Rate = Default_gP_Max_Rate;
    p_PIDObj->gD_Rate = Default_gD_Max_Rate;

    PID_Reset_ProcessVal(p_PIDObj);

    if (RCFilter.init(&p_PIDObj->Dtrim_RC) == 0)
        return false;

    return true;
}

void PID_Reset_ProcessVal(PIDObj_TypeDef *p_PIDObj)
{
    if (p_PIDObj == NULL)
        return;

    p_PIDObj->Integral = 0.0f;
    p_PIDObj->lst_diff = 0.0f;
}

bool PID_Update(PIDObj_TypeDef *p_PIDObj, uint32_t sys_ms, const float mea_in, const float exp_in)
{
    float diff = exp_in - mea_in;
    float out_tmp = 0.0f;

    if (p_PIDObj == NULL)
        return false;
    
    p_PIDObj->in = mea_in;
    p_PIDObj->exp = exp_in;

    /* feed foward is essential? */

    /* if P stage error then PID can`t be use in other progress */
    if (!PID_P_Progress(p_PIDObj, diff))
        return false;

    out_tmp += p_PIDObj->P_out;
    if(PID_I_Progress(p_PIDObj, diff))
        out_tmp += p_PIDObj->I_out;

    if(PID_D_Progress(p_PIDObj, sys_ms, diff))
        out_tmp += p_PIDObj->D_out;

    p_PIDObj->fout = out_tmp;
    return true;
}

static bool PID_P_Progress(PIDObj_TypeDef *p_PIDObj, const float diff)
{
    if (p_PIDObj == NULL)
        return false;
    
    p_PIDObj->P_out = diff * p_PIDObj->gP * p_PIDObj->gP_Rate;
    return true;
}

static bool PID_I_Progress(PIDObj_TypeDef *p_PIDObj, const float diff)
{
    if (p_PIDObj == NULL)
        return false;
        
    p_PIDObj->Integral += diff * p_PIDObj->gI;

    /* limit Integral */
    /* check integer first */
    if (p_PIDObj->gI_Limit)
    {
        if ((int16_t)p_PIDObj->Integral >= (int16_t)p_PIDObj->gI_Max)
        {
            p_PIDObj->Integral = p_PIDObj->gI_Max;
        }
        else if ((int16_t)p_PIDObj->Integral <= (int16_t)p_PIDObj->gI_Min)
        {
            p_PIDObj->Integral = p_PIDObj->gI_Min;
        }
    }

    p_PIDObj->I_out = p_PIDObj->Integral;
    return true;
}

static bool PID_D_Progress(PIDObj_TypeDef *p_PIDObj, uint32_t sys_ms, const float diff)
{
    /* one stage filter is needed */
    /* because high frequence noise on input error will influence D stage */
    float derivative = diff - p_PIDObj->lst_diff;
    // derivative = RCFilter.update((RC_Object_Handle)&(p_PIDObj->Dtrim_RC), derivative);

    if (p_PIDObj == NULL)
        return false;
    
    p_PIDObj->D_out = p_PIDObj->gD * p_PIDObj->gD_Rate * derivative;
    p_PIDObj->lst_diff = diff;
    return true;
}

void PID_P_DynamicTrim(PIDObj_TypeDef *p_PIDObj, bool en, float rate)
{
    if (p_PIDObj == NULL)
        return;

    p_PIDObj->gP_Rate = Default_gP_Max_Rate;
    if (en)
    {
        p_PIDObj->gP_Rate = Default_gP_Min_Rate;
        if (rate > Default_gP_Min_Rate)
            p_PIDObj->gP_Rate = rate;
    }
}

void PID_D_DynamicTrim(PIDObj_TypeDef *p_PIDObj, bool en, float rate)
{
    if (p_PIDObj == NULL)
        return;

    p_PIDObj->gD_Rate = Default_gD_Max_Rate;
    if (en)
    {
        p_PIDObj->gD_Rate = Default_gD_Min_Rate;
        if (rate > Default_gD_Min_Rate)
            p_PIDObj->gD_Rate = rate;
    }
}
