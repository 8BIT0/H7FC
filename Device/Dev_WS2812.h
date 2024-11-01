#ifndef __DEV_WS2812_H
#define __DEV_WS2812_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define WS2812_DATA_SIZE    24
#define WS2812_CLOCK        100000000.0 /* unit: Hz */
#define WS2812_PERIOD       1000        /* unit: ns */

#define WS2812_BLACK        {255, 255, 255}
#define WS2812_GHOSTWHITE   {248, 248, 255}
#define WS2812_SNOWWHITE    {255, 250, 250}

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
    uint8_t num;
    RGB_TypeDef *p_RGB;
    WS2812Bus_Type_List bus;

    uint32_t ctl_data[WS2812_DATA_SIZE];

    void *port_Obj;
    void *(*p_malloc)(uint32_t size);
    void (*p_free)(void *ptr);
    uint8_t (*port_send)(void *port_obj, uint8_t *p_data, uint16_t len);
    bool (*port_init)(void *port_Obj);
} DevWS2812Obj_TypeDef;

typedef struct
{
    bool (*init)(DevWS2812Obj_TypeDef *p_obj);
    bool (*bright)(DevWS2812Obj_TypeDef *p_obj, uint8_t id, uint16_t bright);
    bool (*write)(DevWS2812Obj_TypeDef *obj, uint8_t id,  RGB_TypeDef rgb);
    bool (*reset)(DevWS2812Obj_TypeDef *p_obj);
} DevWS2812_TypeDef;

extern DevWS2812_TypeDef DevWS2812;

#endif
