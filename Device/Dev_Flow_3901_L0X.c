#include "Dev_Flow_3901_L0X.h"

static bool DevFlow3901L0X_Init(DevFlow3901L0XObj_TypeDef *obj)
{
    if(obj && obj->get_timestamp && obj->buf_total_size && obj->p_buf)
    {
        obj->init = true;
        obj->err = DevFlow3901L0X_None_Error;

        obj->buf_cur_size = 0;

        memset(&obj->data, 0, sizeof(obj->data));
        memset(obj->p_buf, 0, obj->buf_total_size);

        return true;
    }

    return false;
}

static bool DevFlow3901L0X_Decode(DevFlow3901L0XObj_TypeDef *obj, uint8_t *p_data, uint16_t size)
{
    if(obj && p_data && size)
    {

    }

    return false;
}

