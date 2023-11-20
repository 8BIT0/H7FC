#ifndef __PID_H
#define __PID_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct
{
    float in;       /* input measurement data in physical */
    float exp;      /* expect physical data */
    float diff_max;
    float diff_min;
    float fout;

    uint16_t accuracy_scale; /* max scale is 10000, noticed it must be the integer power of 10 */

    /* Positional pid */
    int16_t iout_base;
    int16_t iout;
    int16_t iout_min;
    int16_t iout_max;

    /* add member in this section */
    float gP;
    float P_out;
    
    float gI;
    float gI_Min;
    float gI_Max;
    float Integral;
    float I_out;

    float gD;
    float D_out;

    float CTL_period;  /* unit: S */
}PIDObj_TypeDef;


#endif

