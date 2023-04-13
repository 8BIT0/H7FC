#ifndef __FILTER_H
#define __FILTER_H

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "queue.h"

#define SMOOTH_WINDOW_SIZE 5

typedef uint32_t BWF_Object_TypeDef; /* butterworth filter object */

#pragma pack(1)
typedef struct
{
    uint32_t sample_freq;
    uint32_t stop_freq;

    uint8_t order;

    float *e_para_buf;
    float *u_para_buf;

    float *p_e_data_cache;
    float *p_u_data_cache;
}Filter_ButterworthParam_TypeDef;

typedef struct
{
    
}Filter_LowPassParam_TypeDef;

typedef struct
{
    
}Filter_HighPassParam_TypeDef;

typedef struct
{

}Filter_KalmenParam_TypeDef;
#pragma pack()

#endif

