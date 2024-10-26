#ifndef __DEV_WS2812_H
#define __DEV_WS2812_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define RGB_Size sizeof(RGB_TypeDef)

typedef struct
{
    bool en;
    uint16_t bright;

    uint8_t R;
    uint8_t G;
    uint8_t B;
} RGB_TypeDef;

typedef struct
{
    uint8_t num;
    RGB_TypeDef *p_RGB;
    uint8_t (*port_send)(uint8_t *p_data, uint16_t len);
} DevWS2812Obj_TypeDef;

typedef struct
{
    bool (*init)(DevWS2812Obj_TypeDef *p_obj, uint8_t led_num);
    bool (*single_en)(DevWS2812Obj_TypeDef *p_obj, uint8_t id, bool en);
    bool (*single_bright_set)(DevWS2812Obj_TypeDef *p_obj, uint8_t id, uint16_t bright);
    bool (*single_ctl)(DevWS2812Obj_TypeDef *obj, uint8_t id,  RGB_TypeDef rgb);
    bool (*all_en)(DevWS2812Obj_TypeDef *p_obj, bool en);
    bool (*all_ctl)(DevWS2812Obj_TypeDef *p_obj, RGB_TypeDef rgb);
    bool (*all_bright_set)(DevWS2812Obj_TypeDef *p_obj, uint16_t bright);
} DevWS2812_TypeDef;

extern DevWS2812_TypeDef DevWS2812;

#endif
