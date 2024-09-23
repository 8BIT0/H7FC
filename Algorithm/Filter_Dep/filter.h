#ifndef __FILTER_H
#define __FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "linked_list.h"
#include "Srv_OsCommon.h"
#include "filter_param.h"

#define MAX_SMOOTH_WINDOW_SIZE 10
#define FILTER_MALLOC(x) SrvOsCommon.malloc(x)
#define FILTER_FREE(x) SrvOsCommon.free(x);

typedef uint32_t BWF_Object_Handle; /* butterworth filter object */
typedef uint32_t SW_Object_Handle;  /* smooth window filter object */
typedef uint32_t RC_Object_Handle;  /* RC filter object */

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

typedef struct
{
    float r;
    float c;
    float dt;
    float lst_out;
    uint32_t lst_tick;
} RC_Filter_Param_TypeDef;

typedef struct
{
    RC_Object_Handle (*init)(RC_Filter_Param_TypeDef *obj);
    float (*update)(RC_Object_Handle obj, float in);
    float (*get_cut_off)(RC_Object_Handle obj);
} RC_Filter_TypeDef;

typedef struct
{
    uint8_t window_size;
    uint8_t smooth_period;

    list_obj *window_header;
    item_obj *window_ender;
    item_obj *window_cache;
} SmoothWindow_Param_TypeDef;

typedef struct
{
    SW_Object_Handle (*init)(uint8_t window_size);
    float (*update)(SW_Object_Handle hdl, float cur_e);
} SmoothWindow_Filter_TypeDef;

extern Butterworth_Filter_TypeDef Butterworth;
extern SmoothWindow_Filter_TypeDef SmoothWindow;
extern RC_Filter_TypeDef RCFilter;

#ifdef __cplusplus
}
#endif

#endif
