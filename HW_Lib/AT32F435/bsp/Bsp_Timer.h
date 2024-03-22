#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H

#include "Bsp_Timer_Port_Def.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define To_TimerPWMObj_Ptr(x) ((BspTimerPWMObj_TypeDef *)x)

bool BspTimer_SysTick_Init(void);

extern BspTimerPWM_TypeDef BspTimer_PWM;

#endif
