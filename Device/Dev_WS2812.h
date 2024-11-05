#ifndef __DEV_WS2812_H
#define __DEV_WS2812_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define WS2812_DATA_SIZE    24
#define WS2812_CLOCK        144000000.0 /* unit: Hz */
#define WS2812_PERIOD       98          /* unit: ns */
#define WS2812_MAX_BRIGHT   100

#define WS2812_BLACK        (RGB_TypeDef){100, 255, 255, 255}
#define WS2812_GHOSTWHITE   (RGB_TypeDef){100, 248, 248, 255}
#define WS2812_SNOWWHITE    (RGB_TypeDef){100, 255, 250, 250}
#define WS2812_GREEN        (RGB_TypeDef){100, 10,  10,  10}
#define WS2812_NONE         (RGB_TypeDef){100, 0,   0,   0}

#define WS2812_T0H          38
#define WS2812_T1H          75

/* reset period at least 280us */
/* T0L 580ns ~ 1us   -> choose 750ns */
/* T0H 220ns ~ 380ns -> choose 250ns */
/* T1L 220ns ~ 420ns -> choose 250ns */
/* T1H 580ns ~ 1us   -> choose 750ns*/

/*
 *       when bit set
 *|- 750ns -|       |
 *|         |-250ns-|
 * __________       
 * |        |_______
 * ------1us--------
 */

/*
 *       when bit reset
 *|-250ns-|         |
 *|       |- 750ns -|
 * ________       
 * |      |__________
 * ------1us---------
 */

#define RGB_Size sizeof(RGB_TypeDef)
#define To_WS2812Obj_Ptr(x) ((DevWS2812Obj_TypeDef *)x)

typedef enum
{
    WS2812_Bus_Timer = 0,
    WS2812_Bus_Spi,
    WS2812_Bus_Sim,
} WS2812Bus_Type_List;

typedef struct
{
    uint16_t bright;

    uint8_t R;
    uint8_t G;
    uint8_t B;
} RGB_TypeDef;

typedef struct
{
    float H;
    float S;
    float V;
} HSV_TypeDef;

typedef struct
{
    WS2812Bus_Type_List bus;

    RGB_TypeDef RGB;
    HSV_TypeDef HSV;
    uint32_t ctl_data[WS2812_DATA_SIZE];

    void *port_Obj;
    void *(*p_malloc)(uint32_t size);
    void (*p_free)(void *ptr);
    bool (*port_send)(void *port_obj);
    bool (*port_init)(void *port_Obj);
} DevWS2812Obj_TypeDef;

typedef struct
{
    bool (*init)(DevWS2812Obj_TypeDef *p_obj);
    bool (*write)(DevWS2812Obj_TypeDef *p_obj, RGB_TypeDef rgb);
} DevWS2812_TypeDef;

extern DevWS2812_TypeDef DevWS2812;

#endif
