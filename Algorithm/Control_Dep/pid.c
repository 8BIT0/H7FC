/* Author: 8_B!T0
 */
#include "pid.h"

/* internal function */
static bool PID_P_Progress(PIDObj_TypeDef *p_PIDObj, const float diff);
static bool PID_I_Progress(PIDObj_TypeDef *p_PIDObj, const float diff);
static bool PID_D_Progress(PIDObj_TypeDef *p_PIDObj, uint32_t sys_ms, const float diff);

bool PID_Update(PIDObj_TypeDef *p_PIDObj, uint32_t sys_ms, const float mea_in, const float exp_in)
{
    float diff = mea_in - exp_in;
    float out_tmp = 0.0f;

    if(p_PIDObj)
    {
        p_PIDObj->in = mea_in;
        p_PIDObj->exp = exp_in;

        /* feed foward is essential? */

        /* if P stage error then PID can`t be use in other progress */
        if(PID_P_Progress(p_PIDObj, diff))
        {
            out_tmp += p_PIDObj->P_out;
            
            if(PID_I_Progress(p_PIDObj, diff))
            {
                out_tmp += p_PIDObj->I_out;
            }

            if(PID_D_Progress(p_PIDObj, sys_ms, diff))
            {
                out_tmp += p_PIDObj->D_out;
            }

            p_PIDObj->fout = out_tmp;
            /* comput pid integer output down below */

            return true;
        }
    }

    return false;
}

static bool PID_P_Progress(PIDObj_TypeDef *p_PIDObj, const float diff)
{
    float diff_tmp = 0.0f;

    if(p_PIDObj)
    {
        /* limit diff range */
        /* check integer first */
        diff_tmp = diff;
        if((int16_t)diff >= (int16_t)p_PIDObj->diff_max)
        {
            diff_tmp = p_PIDObj->diff_max;
        }
        else if((int16_t)diff <= (int16_t)p_PIDObj->diff_min)
        {
            diff_tmp = p_PIDObj->diff_min;
        }

        p_PIDObj->P_out = diff_tmp * p_PIDObj->gP;
        return true;
    }

    return false;
}

static bool PID_I_Progress(PIDObj_TypeDef *p_PIDObj, const float diff)
{
    if(p_PIDObj)
    {
        p_PIDObj->Integral += diff;

        /* limit Integral */
        /* check integer first */
        if((int16_t)p_PIDObj->Integral >= (int16_t)p_PIDObj->gI_Max)
        {
            p_PIDObj->Integral = p_PIDObj->gI_Max;
        }
        else if((int16_t)p_PIDObj->Integral <= (int16_t)p_PIDObj->gI_Min)
        {
            p_PIDObj->Integral = p_PIDObj->gI_Min;
        }

        p_PIDObj->I_out = p_PIDObj->gI * p_PIDObj->Integral;
        return true;
    }

    return false;
}

static bool PID_D_Progress(PIDObj_TypeDef *p_PIDObj, uint32_t sys_ms, const float diff)
{
    /* one stage filter is needed */
    /* because high frequence noise on input error will influence D stage */
    float derivative = diff - p_PIDObj->lst_diff;
    derivative = RCFilter.update((RC_Object_Handle)&(p_PIDObj->Dtrim_RC), derivative);

    if(p_PIDObj)
    {
        p_PIDObj->D_out = p_PIDObj->gD * derivative;

        p_PIDObj->lst_diff = diff;
        return true;
    }

    return false;
}

