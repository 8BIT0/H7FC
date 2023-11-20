/* Author: 8_B!T0
 *         WX_S
 *         X_X
 */
#include "pid.h"

/* internal function */
static bool PID_P_Progress(PIDObj_TypeDef *p_PIDObj, const float mea_in, const float exp_in);
static bool PID_I_Progress(PIDObj_TypeDef *p_PIDObj);
static bool PID_D_Progress(PIDObj_TypeDef *p_PIDObj);

bool PID_Update(PIDObj_TypeDef *p_PIDObj, const float mea_in, const float exp_in, float *pid_f_out, int16_t *pid_i_out)
{
    bool P_State = false;
    bool I_State = false;
    bool D_State = false;

    if(p_PIDObj && pid_f_out && pid_i_out)
    {
        P_State = PID_P_Progress(p_PIDObj, mea_in, exp_in);
        
        /* if P stage error then PID can`t be use in the other step */
        if(P_State)
        {
            I_State = PID_I_Progress(p_PIDObj);
            D_State = PID_D_Progress(p_PIDObj);

            return true;
        }
    }

    return false;
}

static bool PID_P_Progress(PIDObj_TypeDef *p_PIDObj, const float mea_in, const float exp_in)
{
    float diff = mea_in - exp_in;

    if(p_PIDObj)
    {
        p_PIDObj->P_out = diff * p_PIDObj->gP;
    }

    return false;
}

static bool PID_I_Progress(PIDObj_TypeDef *p_PIDObj)
{
    if(p_PIDObj)
    {

    }

    return false;
}

static bool PID_D_Progress(PIDObj_TypeDef *p_PIDObj)
{
    if(p_PIDObj)
    {

    }

    return false;
}
