#ifndef __PID_H
#define __PID_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct
{
    float in;       /* input measurement data in physical */
    float exp;      /* expect physical data */

    float fout;

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
    float I_out;

    float gD;
    float D_out;

    uint16_t iCTL_period;
    float fCTL_period;
}PIDObj_TypeDef;


#endif

