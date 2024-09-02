#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Bsp_Timer_Port_Def.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define To_Timer_Instance(x) ((tmr_type *)x)
#define To_TimerPWMObj_Ptr(x) ((BspTimerPWMObj_TypeDef *)x)

extern BspTimerPWM_TypeDef BspTimer_PWM;

#ifdef __cplusplus
}
#endif

#endif
