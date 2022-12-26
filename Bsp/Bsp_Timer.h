#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_tim.h"

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
    bool (*init)();
    void (*set_prescaler)();
    void (*set_autoreload)();
    void (*start)();
} BspTimerPWM_TypeDef;

extern BspTimerPWM_TypeDef BspTimer_PWM;

#endif
