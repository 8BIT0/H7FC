#ifndef __FILTER_H
#define __FILTER_H

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "linked_list.h"
#include "Srv_OsCommon.h"
#include "filter_param.h"

#define SMOOTH_WINDOW_SIZE 5
#define FILTER_MALLOC(x) SrvOsCommon.malloc(x)
#define FILTER_FREE(x) SrvOsCommon.free(x);

typedef uint32_t BWF_Object_Handle; /* butterworth filter object */

#pragma pack(1)
typedef struct
{
    uint8_t order;
    BTF_Para_TypeDef *ep_list;
    BTF_Para_TypeDef *up_list;
}FilterParam_Obj_TypeDef;

typedef struct
{
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

#define CREATE_FILTER_PARAM_OBJ(_name ,_order, _stop_freq, _sample_rate, obj_ptr)      \
FilterParam_Obj_TypeDef FilterObj_##_name##_##_order##o_##_stop_freq##_##_sample_rate = { \
    .order = _order,                                                            \
    .ep_list = BTF_E_##_order##O_##_stop_freq##_##_sample_rate,                 \
    .up_list = BTF_U_##_order##O_##_stop_freq##_##_sample_rate                  \
};                                                                              \
obj_ptr = &FilterObj_##_name##_##_order##o_##_stop_freq##_##_sample_rate        \

typedef struct
{
    BWF_Object_Handle (*init)(const FilterParam_Obj_TypeDef *param_obj);
    float (*update)(BWF_Object_Handle obj, float cur_e);
} Butterworth_Filter_TypeDef;

extern Butterworth_Filter_TypeDef Butterworth;

#endif