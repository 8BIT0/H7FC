#include "Dev_WS2812.h"

static bool Dev_WS2812_Init(DevWS2812Obj_TypeDef *p_obj);

DevWS2812_TypeDef DevWS2812 = {
    .init = Dev_WS2812_Init,
};

static bool Dev_WS2812_Init(DevWS2812Obj_TypeDef *p_obj)
{
    if ((p_obj == NULL) || \
        (p_obj->port_init == NULL) || \
        (p_obj->port_send == NULL) || \
        !p_obj->port_init(p_obj))
        return false;

    /* set default color */
    p_obj->RGB = WS2812_GHOSTWHITE;

    return true;
}
