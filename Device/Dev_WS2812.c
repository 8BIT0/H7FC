#include "Dev_WS2812.h"

__attribute__((weak)) void *DevWS2812_Malloc(uint32_t size){return NULL;}
__attribute__((weak)) void DevWS2812_Free(void *ptr){return;}

static bool Dev_WS2812_Init(DevWS2812Obj_TypeDef *p_obj, uint8_t led_num)
{
    if ((p_obj == NULL) || \
        (p_obj->port_send == NULL) || \
        (led_num == 0))
        return false;

    p_obj->num = led_num;
    p_obj->p_RGB = DevWS2812_Malloc(RGB_Size * led_num);
    if (p_obj->RGB == NULL)
    {
        DevWS2812_Free(p_obj->p_RGB);
        return false;
    }

    for (uint8_t i = 0; i < p_obj->num; i ++)
    {
        p_obj->p_RGB[i].en = false;
        p_obj->p_RGB[i].bright = 0;

        /* set default color */
        // p_obj->p_RGB[i].R = ;
        // p_obj->p_RGB[i].G = ;
        // p_obj->p_RGB[i].B = ;
    }

    return true;
}
