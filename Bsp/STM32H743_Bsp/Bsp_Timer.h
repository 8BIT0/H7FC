#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_tim.h"
#include "Bsp_DMA.h"
#include "Bsp_GPIO.h"

typedef void (*BspTimer_Tick_Callback)(const uint32_t tick);

typedef enum
{
    BspTimer_1 = 0,
    BspTimer_2,
    BspTimer_3,
    BspTimer_4,
    BspTimer_5,
    BspTimer_6,
    BspTimer_7,
    BspTimer_8,
    BspTimer_TickObj_Sum,
}BspTimer_Instance_List;

typedef struct
{
    TIM_HandleTypeDef tim_hdl;
    TIM_TypeDef *instance;
    uint32_t prescale;
    uint32_t auto_reload;

    BspTimer_Tick_Callback tick_callback;
}BspTimerTickObj_TypeDef;

typedef struct
{
    TIM_TypeDef *instance;
    uint32_t tim_channel;
    uint8_t dma;
    uint8_t stream;
    DMA_HandleTypeDef dma_hdl;
    TIM_HandleTypeDef tim_hdl;
    uint32_t prescale;
    uint32_t auto_reload;
    uint32_t tim_dma_id_cc;
    uint32_t tim_dma_cc;

    uint32_t buffer_addr;
    uint32_t buffer_size;
} BspTimerPWMObj_TypeDef;

typedef struct
{
    bool (*init)(BspTimerPWMObj_TypeDef *obj,
                 TIM_TypeDef *instance,
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

TIM_HandleTypeDef* BspTimer_Get_Tick_HandlePtr(BspTimer_Instance_List index);

extern BspTimerPWM_TypeDef BspTimer_PWM;
extern BspTimerTick_TypeDef BspTimer_Tick;

#endif