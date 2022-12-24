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
    uint32_t dma_channel;
    TIM_HandleTypeDef hdl;
} BspTimerPWMObj_TypeDef;

typedef struct
{
    bool (*init)();
} BspTimerPWM_TypeDef;

#endif
