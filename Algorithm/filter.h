#ifndef __FILTER_H
#define __FILTER_H

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "linked_list.h"

#define SMOOTH_WINDOW_SIZE 5

typedef uint32_t BWF_Object_Handle; /* butterworth filter object */

#pragma pack(1)
typedef struct
{
    uint32_t sample_freq;
    uint32_t stop_freq;

    uint8_t order;

    float *e_para_buf;
    float *u_para_buf;

    list_obj *p_e_list_header;
    item_obj *p_e_list_ender;
    list_obj *p_u_list_header;
    item_obj *p_u_list_ender;

    item_obj *p_e_data_cache;
    item_obj *p_u_data_cache;
} Filter_ButterworthParam_TypeDef;
#pragma pack()

typedef struct
{
    BWF_Object_Handle (*init)(uint32_t sample_freq, uint8_t stop_freq, uint8_t order, float *e_para, float *u_para);
    float (*update)(BWF_Object_Handle obj, float cur_e);
} Butterworth_Filter_TypeDef;

extern Butterworth_Filter_TypeDef Butterworth;

#endif
