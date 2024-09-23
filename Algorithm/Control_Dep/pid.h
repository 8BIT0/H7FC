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
    float base_diff;

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
    float diff_max;
    float diff_min;
    float fout;

    /* add member in this section */
    float gP;
    float P_out;
    
    float gI;
    float gI_Min;
    float gI_Max;
    float Integral;
    float I_out;

    float gD;
    float lst_diff;
    float D_out;
    RC_Filter_Param_TypeDef Dtrim_RC;
}PIDObj_TypeDef;

bool PID_Update(PIDObj_TypeDef *p_PIDObj, uint32_t sys_ms, const float mea_in, const float exp_in);

#ifdef __cplusplus
}
#endif

#endif

