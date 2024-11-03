#include "Dev_WS2812.h"

static bool Dev_WS2812_Init(DevWS2812Obj_TypeDef *p_obj);
static bool Dev_WS2812_Write(DevWS2812Obj_TypeDef *p_obj, RGB_TypeDef rgb);

DevWS2812_TypeDef DevWS2812 = {
    .init = Dev_WS2812_Init,
    .write = Dev_WS2812_Write,
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

static bool Dev_WS2812_Write(DevWS2812Obj_TypeDef *p_obj, RGB_TypeDef rgb)
{
    uint8_t data[3] = {0};
    uint8_t bit = 0x00;

    if ((p_obj == NULL) || \
        (p_obj->port_send == NULL))
        return false;

    p_obj->RGB = rgb;
    data[0] = p_obj->RGB.G;
    data[1] = p_obj->RGB.R;
    data[2] = p_obj->RGB.B;

    for (uint8_t i = 0; i < WS2812_DATA_SIZE; i ++)
    {
        bit = 0x00;
        p_obj->ctl_data[i] = WS2812_T0H;
        bit |= 1 << (7 - (i % 8));
        if (data[i / 8] & bit)
            p_obj->ctl_data[i] = WS2812_T1H;
    }

    return p_obj->port_send(p_obj->port_Obj);
}

