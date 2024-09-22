/*
 * we use tcm section to increase the speed on filter algorithm function execution
 */

#include "filter.h"

#define PI 3.14159265359

/* internal function */
static void Filter_Item_Update(item_obj **header, item_obj **ender, float cur_data);
static bool Filter_List_Create(uint8_t order, item_obj *item, list_obj **header, item_obj **ender);

/* external function */
/* butterworth filter section */
static BWF_Object_Handle Butterworth_Init(const FilterParam_Obj_TypeDef *param_obj);
static float Butterworth_Filter_Update(BWF_Object_Handle obj, float cur_e);
/* smooth window filter section */
static SW_Object_Handle SmoothWindow_Init(uint8_t window_size);
static float SmoothWindow_Update(SW_Object_Handle hdl, float cur_e);
/* rc filter section */
static RC_Object_Handle RC_Filter_Init(RC_Filter_Param_TypeDef *obj);
static float RC_Filter_Update(RC_Object_Handle hdl, uint32_t sys_ms, float in);
static float RC_Filter_Get_Cut_Off(RC_Object_Handle hdl);

Butterworth_Filter_TypeDef Butterworth = {
    .init = Butterworth_Init,
    .update = Butterworth_Filter_Update,
};

SmoothWindow_Filter_TypeDef SmoothWindow = {
    .init = SmoothWindow_Init,
    .update = SmoothWindow_Update,
};

RC_Filter_TypeDef RCFilter = {
    .init = RC_Filter_Init,
    .update = RC_Filter_Update,
    .get_cut_off = RC_Filter_Get_Cut_Off,
};

/********************************************************** general filter section *********************************************************/
static bool Filter_List_Create(uint8_t order, item_obj *item, list_obj **header, item_obj **ender)
{
    for (uint8_t i = 0; i < order; i++)
    {
        float *u_temp = (float *)FILTER_MALLOC(sizeof(float));

        if (u_temp == NULL)
        {
            FILTER_FREE(u_temp);
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
            *header = &item[0];
    }

    *ender = &item[order - 1];

    return true;
}

static void Filter_Item_Update(item_obj **header, item_obj **ender, float cur_data)
{
    item_obj *i_tmp = NULL;

    *((float *)((*ender)->data)) = cur_data;
    (*ender)->prv->nxt = NULL;
    i_tmp = (*ender)->prv;
    (*ender)->prv = NULL;
    (*header)->prv = *ender;
    (*ender)->nxt = *header;
    *header = *ender;
    *ender = i_tmp;
}

/********************************************************** butterworth filter section *********************************************************/
static BWF_Object_Handle Butterworth_Init(const FilterParam_Obj_TypeDef *param_obj)
{
    Filter_ButterworthParam_TypeDef *BWF_Obj = NULL;
    uint8_t e_cnt = 0;
    uint8_t u_cnt = 0;

    if(param_obj == NULL)
        return 0;

    if (param_obj->order > 1)
    {
        e_cnt = param_obj->order + 1;
        u_cnt = param_obj->order;
        
        if ((param_obj->ep_list == NULL) || (param_obj->up_list == NULL))
            return 0;

        BWF_Obj = FILTER_MALLOC(sizeof(Filter_ButterworthParam_TypeDef));

        if (BWF_Obj == NULL)
        {
            FILTER_FREE(BWF_Obj);
            return 0;
        }

        BWF_Obj->p_e_data_cache = FILTER_MALLOC(sizeof(item_obj) * e_cnt);
        if (BWF_Obj->p_e_data_cache == NULL)
        {
            FILTER_FREE(BWF_Obj->p_e_data_cache);
            FILTER_FREE(BWF_Obj);
            return 0;
        }
        else
        {
            if (!Filter_List_Create(e_cnt, BWF_Obj->p_e_data_cache, &(BWF_Obj->p_e_list_header), &(BWF_Obj->p_e_list_ender)))
            {
                FILTER_FREE(BWF_Obj->p_e_data_cache);
                FILTER_FREE(BWF_Obj);
                return 0;
            }
        }

        BWF_Obj->p_u_data_cache = FILTER_MALLOC(sizeof(item_obj) * u_cnt);
        if (BWF_Obj->p_u_data_cache == NULL)
        {
            FILTER_FREE(BWF_Obj->p_e_data_cache);
            FILTER_FREE(BWF_Obj->p_u_data_cache);
            FILTER_FREE(BWF_Obj);
            return 0;
        }
        else
        {
            if (!Filter_List_Create(u_cnt, BWF_Obj->p_u_data_cache, &(BWF_Obj->p_u_list_header), &(BWF_Obj->p_u_list_ender)))
            {
                FILTER_FREE(BWF_Obj->p_e_data_cache);
                FILTER_FREE(BWF_Obj->p_u_data_cache);
                FILTER_FREE(BWF_Obj);
                return 0;
            }
        }

        BWF_Obj->e_para_buf = FILTER_MALLOC(sizeof(float) * e_cnt);
        if(BWF_Obj->e_para_buf == NULL)
        {
            FILTER_FREE(BWF_Obj->e_para_buf);
            FILTER_FREE(BWF_Obj->p_e_data_cache);
            FILTER_FREE(BWF_Obj->p_u_data_cache);
            FILTER_FREE(BWF_Obj);
            return 0;
        }

        BWF_Obj->u_para_buf = FILTER_MALLOC(sizeof(float) * u_cnt);
        if(BWF_Obj->u_para_buf == NULL)
        {
            FILTER_FREE(BWF_Obj->e_para_buf);
            FILTER_FREE(BWF_Obj->u_para_buf);
            FILTER_FREE(BWF_Obj->p_e_data_cache);
            FILTER_FREE(BWF_Obj->p_u_data_cache);
            FILTER_FREE(BWF_Obj);
            return 0;
        }

        BWF_Obj->order = param_obj->order;

        for(uint8_t i = 0; i < e_cnt; i++)
        {
            BWF_Obj->e_para_buf[i] = param_obj->ep_list[i].p * param_obj->ep_list[i].scale;
        
            if(i < u_cnt)
            {
                BWF_Obj->u_para_buf[i] = param_obj->up_list[i].p * param_obj->up_list[i].scale;
            }
        }
    }

    return (uint32_t)BWF_Obj;
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
        Filter_Item_Update(&(filter_obj->p_e_list_header), &(filter_obj->p_e_list_ender), cur_e);

        u_item = filter_obj->p_u_list_header;
        e_item = filter_obj->p_e_list_header;

        for (uint8_t i = 0; i <= filter_obj->order; i++)
        {
            /* comput E additive */
            if (e_item)
            {
                E_Additive += filter_obj->e_para_buf[i] * (*(float *)(e_item->data));
                e_item = e_item->nxt;
            }

            /* comput U additive */
            if (i < filter_obj->order && u_item)
            {
                U_Additive += filter_obj->u_para_buf[i] * (*(float *)(u_item->data));
                u_item = u_item->nxt;
            }
        }

        u_tmp = E_Additive - U_Additive;
        /* update last time filted data */
        Filter_Item_Update(&(filter_obj->p_u_list_header), &(filter_obj->p_u_list_ender), u_tmp);
    }

    return u_tmp;
}

/********************************************************** smooth window filter section *********************************************************/
static SW_Object_Handle SmoothWindow_Init(uint8_t window_size)
{
    SW_Object_Handle hdl = 0;
    SmoothWindow_Param_TypeDef *param = NULL;

    if(window_size <= MAX_SMOOTH_WINDOW_SIZE)
    {
        param = FILTER_MALLOC(sizeof(SmoothWindow_Param_TypeDef));

        if(param == NULL)
            return 0;

        param->smooth_period = 0;
        param->window_size = window_size;
        param->window_cache = FILTER_MALLOC(sizeof(item_obj) * window_size);

        if(param->window_cache == NULL)
        {
            FILTER_FREE(param->window_cache);
            FILTER_FREE(param);
            return 0;
        }

        if(!Filter_List_Create(window_size, param->window_cache, &(param->window_header), &(param->window_ender)))
            return 0;

        hdl = (uint32_t)param;
    }

    return hdl;
}

static int SmoothWindow_Comput_Sum(item_obj *item, void *item_data, void *sum)
{
    if(item && item_data && sum)
    {
        *((float *) sum) += *((float *) item_data);

        return 1;
    }

    return 0;
}

static float SmoothWindow_Update(SW_Object_Handle hdl, float cur_e)
{
    float sum = 0.0f;
    SmoothWindow_Param_TypeDef *param = NULL; 

    if(hdl)
    {
        param = (SmoothWindow_Param_TypeDef *)hdl;
    
        if(param->smooth_period < param->window_size)
            param->smooth_period ++;

        Filter_Item_Update(&(param->window_header), &(param->window_ender), cur_e);
     
        List_traverse(param->window_header, SmoothWindow_Comput_Sum, &sum, pre_callback);
        sum /= param->smooth_period;
    }

    return sum;
}

/********************************************************** RC Filter section *********************************************************/
static RC_Object_Handle RC_Filter_Init(RC_Filter_Param_TypeDef *obj)
{
    RC_Object_Handle hdl = 0;

    if (obj == NULL)
        return 0;

    return hdl;
}

static float RC_Filter_Update(RC_Object_Handle hdl, uint32_t sys_ms, float in)
{
    float val = 0.0f;

    if (hdl == 0)
        return 0.0f;

    return val;
}

static float RC_Filter_Get_Cut_Off(RC_Object_Handle hdl)
{
    float cut_off = 0.0f;

    if (hdl == 0)
        return 0.0f;

    return cut_off;
}

