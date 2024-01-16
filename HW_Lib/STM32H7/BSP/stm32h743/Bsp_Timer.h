#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H

#include <stdint.h>
#include <stdbool.h>
#include "Bsp_Timer_Port_Def.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_tim.h"
#include "Bsp_DMA.h"
#include "Bsp_GPIO.h"

typedef void (*BspTimer_Tick_Callback)(const uint32_t tick);

#define TIM_HandleType_Size sizeof(TIM_HandleTypeDef)
#define TIM_DMA_HandleType_Size sizeof(DMA_HandleTypeDef)

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

TIM_HandleTypeDef* BspTimer_Get_Tick_HandlePtr(BspTimer_Instance_List index);

extern BspTimerPWM_TypeDef BspTimer_PWM;
extern BspTimerTick_TypeDef BspTimer_Tick;

#endif
