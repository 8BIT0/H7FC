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

static bool Butterworth_List_Create(uint8_t order, item_obj *item, list_obj *header, item_obj *ender)
{
    for(uint8_t i = 0; i < order; i++)
    {
        float *u_temp = (float *)MMU_Malloc(sizeof(float));

        if(u_temp == NULL)
        {
            MMU_Free(u_temp);
            return false;
        }
        else
        {
            List_ItemInit(&item[i], u_temp);
            *u_temp = 0.0f;
        }

        /* set first element as list head */
        if(i > 0)
        {
            item[i - 1].nxt = &item[i];
        }
        else
            header = &item[0];
    }

    ender = &item[order - 1];

    return true;
}

static BWF_Object_TypeDef Butterworth_Init(uint32_t sample_freq, uint8_t stop_freq, uint8_t order, float *e_para, float *u_para)
{
    Filter_ButterworthParam_TypeDef *BWF_Obj = NULL;
    uint8_t e_cnt = order;
    uint8_t u_cnt = order - 1;


    if(sample_freq && (order > 1))
    {
        if((e_para == NULL) || (u_para == NULL))
            return 0;

        BWF_Obj = MMU_Malloc(sizeof(Filter_ButterworthParam_TypeDef));
        
        if(BWF_Obj == NULL)
        {
            MMU_Free(BWF_Obj);
            return 0;
        }

        BWF_Obj->p_e_data_cache = MMU_Malloc(sizeof(item_obj) * e_cnt);
        if(BWF_Obj->p_e_data_cache == NULL)
        {
            MMU_Free(BWF_Obj);
            MMU_Free(BWF_Obj->p_e_data_cache);
            return 0;
        }
        else
        {
            if(!Butterworth_List_Create(e_cnt, BWF_Obj->p_e_data_cache, BWF_Obj->p_e_list_header, BWF_Obj->p_e_list_ender))
            {
                MMU_Free(BWF_Obj);
                MMU_Free(BWF_Obj->p_e_data_cache);
                return 0;
            }
        }

        BWF_Obj->p_u_data_cache = MMU_Malloc(sizeof(item_obj) * u_cnt);
        if(BWF_Obj->p_u_data_cache == NULL)
        {
            MMU_Free(BWF_Obj);
            MMU_Free(BWF_Obj->p_e_data_cache);
            MMU_Free(BWF_Obj->p_u_data_cache);
            return 0;
        }
        else
        {
            if(!Butterworth_List_Create(u_cnt, BWF_Obj->p_u_data_cache, BWF_Obj->p_u_list_header, BWF_Obj->p_u_list_ender))
            {
                MMU_Free(BWF_Obj);
                MMU_Free(BWF_Obj->p_e_data_cache);
                MMU_Free(BWF_Obj->p_u_data_cache);
                return 0;
            }
        }

        BWF_Obj->stop_freq = stop_freq;
        BWF_Obj->sample_freq = sample_freq;
        BWF_Obj->order = order;

        BWF_Obj->e_para_buf = e_para;
        BWF_Obj->u_para_buf = u_para;

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
