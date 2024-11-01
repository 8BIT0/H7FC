#include "Dev_WS2812.h"

static bool Dev_WS2812_Init(DevWS2812Obj_TypeDef *p_obj);

DevWS2812_TypeDef DevWS2812 = {
    .init = Dev_WS2812_Init,
};

static bool Dev_WS2812_Init(DevWS2812Obj_TypeDef *p_obj)
{
    if ((p_obj == NULL) || \
        (p_obj->num == 0) || \
        (p_obj->port_init == NULL) || \
        (p_obj->port_send == NULL) || \
        (p_obj->p_malloc == NULL) || \
        (p_obj->p_free == NULL) || \
        !p_obj->port_init(p_obj))
        return false;

    p_obj->p_RGB = p_obj->p_malloc(RGB_Size * p_obj->num);
    if (p_obj->p_RGB == NULL)
    {
        p_obj->p_free(p_obj->p_RGB);
        return false;
    }

    for (uint8_t i = 0; i < p_obj->num; i ++)
    {
        p_obj->p_RGB[i].bright = 0;

        /* set default color */
        // p_obj->p_RGB[i].R = ;
        // p_obj->p_RGB[i].G = ;
        // p_obj->p_RGB[i].B = ;
    }

    return true;
}
