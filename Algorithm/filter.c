/*
 * we use tcm section to increase the speed on filter algorithm function execution
 */

#include "filter.h"
#include "system_cfg.h"
#include "mmu.h"

/* external function */
static BWF_Object_TypeDef Butterworth_Init(uint32_t sample_freq, uint8_t stop_freq, uint8_t order, float *e_para, float *u_para);
static float Butterworth_Filter_Update(BWF_Object_TypeDef obj, float cur_e);

Butterworth_Filter_TypeDef Butterworth = {
    .init = Butterworth_Init,
    .update = Butterworth_Filter_Update,
};

static BWF_Object_TypeDef Butterworth_Init(uint32_t sample_freq, uint8_t stop_freq, uint8_t order, float *e_para, float *u_para)
{
    Filter_ButterworthParam_TypeDef *BWF_Obj = NULL;

    if(sample_freq && order)
    {
        if((e_para == NULL) || (u_para == NULL))
            return 0;

        BWF_Obj = MMU_Malloc(sizeof(Filter_ButterworthParam_TypeDef));
        
        if(BWF_Obj == NULL)
            return 0;

        BWF_Obj->p_e_data_cache = MMU_Malloc(sizeof(float) * order);
        if(BWF_Obj->p_e_data_cache == NULL)
        {
            MMU_Free(BWF_Obj->p_e_data_cache);
            return 0;
        }
        memset(BWF_Obj->p_e_data_cache, 0, sizeof(float) * order);

        BWF_Obj->p_u_data_cache = MMU_Malloc(sizeof(float) * order);
        if(BWF_Obj->p_u_data_cache == NULL)
        {
            MMU_Free(BWF_Obj->p_u_data_cache);
            return 0;
        }
        memset(BWF_Obj->p_u_data_cache, 0, sizeof(float) * order);

        BWF_Obj->stop_freq = stop_freq;
        BWF_Obj->sample_freq = sample_freq;
        BWF_Obj->order = order;

        BWF_Obj->e_para_buf = e_para;
        BWF_Obj->u_para_buf = u_para;

        /* Use Queue !!!!!!!!! */

        return (uint32_t)BWF_Obj;
    }

    return 0;
}

static float Butterworth_Filter_Update(BWF_Object_TypeDef obj, float cur_e)
{
    float u_tmp = 0.0f;
    float E_Additive = 0.0f;
    float U_Additive = 0.0f;

    if(obj)
    {
        
    }

    return u_tmp;
}
