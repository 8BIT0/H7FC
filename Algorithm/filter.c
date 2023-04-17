/*
 * we use tcm section to increase the speed on filter algorithm function execution
 */

#include "filter.h"
#include "system_cfg.h"
#include "mmu.h"

/* internal function */
static bool Butterworth_List_Create(uint8_t order, item_obj *item, list_obj *header, item_obj *ender);
static void Butterworth_Item_Update(item_obj *header, item_obj *ender, float cur_data);

/* external function */
static BWF_Object_Handle Butterworth_Init(uint32_t sample_freq, uint8_t stop_freq, uint8_t order, float *e_para, float *u_para);
static float Butterworth_Filter_Update(BWF_Object_Handle obj, float cur_e);

Butterworth_Filter_TypeDef Butterworth = {
    .init = Butterworth_Init,
    .update = Butterworth_Filter_Update,
};

static bool Butterworth_List_Create(uint8_t order, item_obj *item, list_obj *header, item_obj *ender)
{
    for (uint8_t i = 0; i < order; i++)
    {
        float *u_temp = (float *)MMU_Malloc(sizeof(float));

        if (u_temp == NULL)
        {
            MMU_Free(u_temp);
            return false;
        }
        else
        {
            List_ItemInit(&item[i], u_temp);
            *u_temp = 0.0f;
        }

        /* link element */
        if (i > 0)
        {
            item[i - 1].nxt = &item[i];
            item[i].prv = &item[i - 1];
        }
        else
            header = &item[0];
    }

    ender = &item[order - 1];

    return true;
}

static BWF_Object_Handle Butterworth_Init(uint32_t sample_freq, uint8_t stop_freq, uint8_t order, float *e_para, float *u_para)
{
    Filter_ButterworthParam_TypeDef *BWF_Obj = NULL;
    uint8_t e_cnt = order + 1;
    uint8_t u_cnt = order;

    if (sample_freq && (order > 1))
    {
        if ((e_para == NULL) || (u_para == NULL))
            return 0;

        BWF_Obj = MMU_Malloc(sizeof(Filter_ButterworthParam_TypeDef));

        if (BWF_Obj == NULL)
        {
            MMU_Free(BWF_Obj);
            return 0;
        }

        BWF_Obj->p_e_data_cache = MMU_Malloc(sizeof(item_obj) * e_cnt);
        if (BWF_Obj->p_e_data_cache == NULL)
        {
            MMU_Free(BWF_Obj);
            MMU_Free(BWF_Obj->p_e_data_cache);
            return 0;
        }
        else
        {
            if (!Butterworth_List_Create(e_cnt, BWF_Obj->p_e_data_cache, BWF_Obj->p_e_list_header, BWF_Obj->p_e_list_ender))
            {
                MMU_Free(BWF_Obj);
                MMU_Free(BWF_Obj->p_e_data_cache);
                return 0;
            }
        }

        BWF_Obj->p_u_data_cache = MMU_Malloc(sizeof(item_obj) * u_cnt);
        if (BWF_Obj->p_u_data_cache == NULL)
        {
            MMU_Free(BWF_Obj);
            MMU_Free(BWF_Obj->p_e_data_cache);
            MMU_Free(BWF_Obj->p_u_data_cache);
            return 0;
        }
        else
        {
            if (!Butterworth_List_Create(u_cnt, BWF_Obj->p_u_data_cache, BWF_Obj->p_u_list_header, BWF_Obj->p_u_list_ender))
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

static void Butterworth_Item_Update(item_obj *header, item_obj *ender, float cur_data)
{
    item_obj *i_tmp = NULL;

    *((float *)(ender->data)) = cur_data;
    ender->prv->nxt = NULL;
    i_tmp = ender->prv;
    ender->prv = NULL;
    header->prv = ender;
    ender->nxt = header;
    header = ender;
    ender = i_tmp;
}

static float Butterworth_Filter_Update(BWF_Object_Handle obj, float cur_e)
{
    Filter_ButterworthParam_TypeDef *filter_obj = NULL;
    item_obj *u_item = NULL;
    item_obj *e_item = NULL;

    float u_tmp = 0.0f;
    float E_Additive = 0.0f;
    float U_Additive = 0.0f;

    if (obj)
    {
        filter_obj = (Filter_ButterworthParam_TypeDef *)obj;
        Butterworth_Item_Update(filter_obj->p_e_list_header, filter_obj->p_e_list_ender, cur_e);

        u_item = filter_obj->p_u_list_header;
        e_item = filter_obj->p_e_list_header;

        for (uint8_t i = 0; i < filter_obj->order + 1; i++)
        {
            /* comput E additive */
            if (e_item)
            {
                E_Additive += filter_obj->e_para_buf[i] * (*(float *)(e_item->data));
                e_item = e_item->nxt;
            }

            /* comput U additive */
            if (i <= filter_obj->order && u_item)
            {
                U_Additive += filter_obj->u_para_buf[i] * (*(float *)(u_item->data));
                u_item = u_item->nxt;
            }
        }

        u_tmp = E_Additive - U_Additive;
        /* update last time filted data */
        Butterworth_Item_Update(filter_obj->p_u_list_header, filter_obj->p_u_list_ender, u_tmp);
    }

    return u_tmp;
}
