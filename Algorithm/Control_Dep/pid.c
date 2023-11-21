/* Author: 8_B!T0
 *         WX_S
 */
#include "pid.h"

/* internal function */
static bool PID_Accuracy_Check(uint16_t accuracy);
static bool PID_P_Progress(PIDObj_TypeDef *p_PIDObj, const float diff);
static bool PID_I_Progress(PIDObj_TypeDef *p_PIDObj, const float diff);
static bool PID_D_Progress(PIDObj_TypeDef *p_PIDObj, const float diff);

bool PID_Update(PIDObj_TypeDef *p_PIDObj, const float mea_in, const float exp_in, float *pid_f_out, int16_t *pid_i_out)
{
    bool P_State = false;
    bool I_State = false;
    bool D_State = false;
    float diff = mea_in - exp_in;
    float out_tmp = 0.0f;

    if(p_PIDObj && PID_Accuracy_Check(p_PIDObj->accuracy_scale) && pid_f_out && pid_i_out)
    {
        p_PIDObj->in = mea_in;
        p_PIDObj->exp = exp_in;

        P_State = PID_P_Progress(p_PIDObj, diff);
        
        /* if P stage error then PID can`t be use in other progress */
        if(P_State)
        {
            out_tmp += p_PIDObj->P_out;
            
            I_State = PID_I_Progress(p_PIDObj, diff);
            D_State = PID_D_Progress(p_PIDObj, diff);

            if(I_State)
            {
                out_tmp += p_PIDObj->I_out;
            }

            if(D_State)
            {
                out_tmp += p_PIDObj->D_out;
            }

            (*pid_f_out) = out_tmp;
            /* comput pid integer output down below */

            return true;
        }
    }

    return false;
}

static bool PID_P_Progress(PIDObj_TypeDef *p_PIDObj, const float diff)
{
    float diff_tmp = 0.0f;
    int16_t diff_fractional = 0;
    int16_t max_fractional = 0;
    int16_t min_fractional = 0;

    if(p_PIDObj && (int16_t)(p_PIDObj->diff_max * p_PIDObj->accuracy_scale) && (int16_t)(p_PIDObj->diff_min * p_PIDObj->accuracy_scale))
    {
        diff_fractional = (diff - (int16_t)diff) * p_PIDObj->accuracy_scale;

        /* limit diff range */
        /* check integer first */
        if((int16_t)diff >= (int16_t)p_PIDObj->diff_max)
        {
            /* check fractional part */
            if((int16_t)diff == (int16_t)p_PIDObj->diff_max)
            {
                max_fractional = (p_PIDObj->diff_max - (int16_t)p_PIDObj->diff_max) * p_PIDObj->accuracy_scale;
                if(diff_fractional > max_fractional)
                {
                    diff_tmp = p_PIDObj->diff_max;
                }
            }
            else
                diff_tmp = p_PIDObj->diff_max;
        }
        else if((int16_t)diff <= (int16_t)p_PIDObj->diff_min)
        {
            /* check fractional part */
            if((int16_t)diff == (int16_t)p_PIDObj->diff_min)
            {
                min_fractional = (p_PIDObj->diff_min - (int16_t)p_PIDObj->diff_min) * p_PIDObj->accuracy_scale;
                if(diff_fractional < min_fractional)
                {
                    diff_tmp = p_PIDObj->diff_min;
                }
            }
            else
                diff_tmp = p_PIDObj->diff_min;
        }
        else
            diff_tmp = diff;

        p_PIDObj->P_out = diff_tmp * p_PIDObj->gP;
        return true;
    }

    return false;
}

static bool PID_I_Progress(PIDObj_TypeDef *p_PIDObj, const float diff)
{
    int16_t gI_Max_Fractical = 0.0f;
    int16_t gI_Min_Fractical = 0.0f;
    int16_t diff_Fractical = 0.0f;

    if(p_PIDObj)
    {
        p_PIDObj->Integral += diff;

        /* limit Integral */
        /* check integer first */
        if((int16_t)p_PIDObj->Integral >= (int16_t)p_PIDObj->gI_Max)
        {
            if((int16_t)p_PIDObj->Integral == (int16_t)p_PIDObj->gI_Max)
            {
                gI_Max_Fractical = (p_PIDObj->gI_Max - (int16_t)p_PIDObj->gI_Max) * p_PIDObj->accuracy_scale;
                diff_Fractical = (p_PIDObj->Integral - (int16_t)p_PIDObj->gI_Max) * p_PIDObj->accuracy_scale;

                /* check Fractional Part */
                if(diff_Fractical > gI_Max_Fractical)
                {
                    p_PIDObj->Integral = p_PIDObj->gI_Max;
                }
            }
            else
                p_PIDObj->Integral = p_PIDObj->gI_Max;
        }
        else if((int16_t)p_PIDObj->Integral <= (int16_t)p_PIDObj->gI_Min)
        {
            if((int16_t)p_PIDObj->Integral == (int16_t)p_PIDObj->gI_Min)
            {
                gI_Min_Fractical = (p_PIDObj->gI_Min - (int16_t)p_PIDObj->gI_Min) * p_PIDObj->accuracy_scale;
                diff_Fractical = (p_PIDObj->Integral - (int16_t)p_PIDObj->gI_Min) * p_PIDObj->accuracy_scale;
                
                /* check Fractional Part */
                if(diff_Fractical < gI_Min_Fractical)
                {
                    p_PIDObj->Integral = p_PIDObj->gI_Min;
                }
            }
            else
                p_PIDObj->Integral = p_PIDObj->gI_Min;
        }

        p_PIDObj->I_out = p_PIDObj->gI * p_PIDObj->Integral;
        return true;
    }

    return false;
}

static bool PID_D_Progress(PIDObj_TypeDef *p_PIDObj, const float diff)
{
    if(p_PIDObj)
    {
        p_PIDObj->D_out = p_PIDObj->gD * (diff - p_PIDObj->lst_diff);

        p_PIDObj->lst_diff = diff;
        return true;
    }

    return false;
}

static bool PID_Accuracy_Check(uint16_t accuracy)
{
    if(accuracy)
    {
        if((accuracy % 10) || (accuracy % 100) || (accuracy % 1000) || (accuracy % 10000) || \
           (accuracy / 10) || (accuracy / 100) || (accuracy / 1000) || (accuracy / 10000))
            return false;

        return true;
    }

    return false;
}
