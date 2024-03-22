#ifndef __BSP_TIMER_PORT_DEF_H
#define __BSP_TIMER_PORT_DEF_H

#include <stdint.h>
#include <stdbool.h>
#include "Bsp_GPIO_Port_Def.h"

typedef void (*BspTimer_Tick_Callback)(const uint32_t tick);

typedef struct
{
    void *tim_hdl;
    void *instance;
    uint32_t prescale;
    uint32_t auto_reload;

    BspTimer_Tick_Callback tick_callback;
}BspTimerTickObj_TypeDef;

typedef struct
{
    void *instance;
    uint32_t tim_channel;
    uint8_t dma;
    uint8_t stream;
    void *dma_hdl;
    void *tim_hdl;
    uint32_t prescale;
    uint32_t auto_reload;
    uint32_t tim_dma_id_cc;
    uint32_t tim_dma_cc;

    uint32_t buffer_addr;
    uint32_t buffer_size;

#if defined AT32F435RGT7
    void *dma_callback_obj;
#endif
} BspTimerPWMObj_TypeDef;

typedef struct
{
    bool (*init)(BspTimerPWMObj_TypeDef *obj,
                 void *instance,
                 uint32_t ch,
                 BspGPIO_Obj_TypeDef pin,
                 uint8_t dma,
                 uint8_t stream,
                 uint32_t buf_aadr,
                 uint32_t buf_size);
    void (*set_prescaler)(BspTimerPWMObj_TypeDef *obj, uint32_t prescale);
    void (*set_autoreload)(BspTimerPWMObj_TypeDef *obj, uint32_t autoreload);
    void (*start_pwm)(BspTimerPWMObj_TypeDef *obj);
    void (*dma_trans)(BspTimerPWMObj_TypeDef *obj);
    uint32_t (*get_clock_freq)(BspTimerPWMObj_TypeDef *obj);
} BspTimerPWM_TypeDef;

typedef struct
{
    bool (*init)(BspTimerTickObj_TypeDef *obj, uint32_t perscale, uint32_t period);
    void (*set_callback)(BspTimerTickObj_TypeDef *obj, BspTimer_Tick_Callback cb);
    bool (*start)(BspTimerTickObj_TypeDef *obj);
    bool (*stop)(BspTimerTickObj_TypeDef *obj);
    bool (*reset)(BspTimerTickObj_TypeDef *obj);
    void (*check_counter)(BspTimerTickObj_TypeDef *obj);
    void (*trim_reload)(BspTimerTickObj_TypeDef *obj, uint32_t reload_val);
    void (*trim_counter)(BspTimerTickObj_TypeDef *obj, uint32_t counter_val);
}BspTimerTick_TypeDef;

#endif
