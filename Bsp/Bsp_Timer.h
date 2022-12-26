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
    void (*start)();
} BspTimerPWM_TypeDef;

extern BspTimerPWM_TypeDef BspTimer_PWM;

#endif
