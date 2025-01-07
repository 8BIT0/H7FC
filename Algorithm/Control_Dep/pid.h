#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "filter.h"

typedef struct
{
    float gP;

    float gI;
    float gI_Max;
    float gI_Min;
    
    float gD;
} PID_Param_TypeDef;

typedef struct
{
    float in;       /* input measurement data in physical */
    float exp;      /* expect physical data */
    float fout;

    /* add member in this section */
    float gP;
    float gP_Rate;
    float P_out;
    
    float gI;
    bool  gI_Limit;
    float gI_Min;
    float gI_Max;
    float Integral;
    float I_out;

    float gD;
    float gD_Rate;
    float lst_diff;
    float D_out;
    RC_Filter_Param_TypeDef Dtrim_RC;
}PIDObj_TypeDef;

bool PID_Init(PIDObj_TypeDef *p_PIDObj, RC_Filter_Param_TypeDef rc_para);
void PID_Reset_ProcessVal(PIDObj_TypeDef *p_PIDObj);
bool PID_Update(PIDObj_TypeDef *p_PIDObj, uint32_t sys_ms, const float mea_in, const float exp_in);
void PID_P_DynamicTrim(PIDObj_TypeDef *p_PIDObj, bool en, float rate);
void PID_D_DynamicTrim(PIDObj_TypeDef *p_PIDObj, bool en, float rate);

#ifdef __cplusplus
}
#endif

#endif

