#ifndef __DEBUG_UTIL_H
#define __DEBUG_UTIL_H

#include <stdbool.h>
#include <stdint.h>
#include "Bsp_GPIO.h"

void assert(bool state);

#pragma pack(1)
typedef BspGPIO_OutPutObj_TypeDef DebugPinObj_TypeDef;
#pragma pack()

typedef struct
{
    bool (*init)(DebugPinObj_TypeDef pin);
    bool (*ctl)(DebugPinObj_TypeDef pin, bool state);
} DebugPin_TypeDef;

extern DebugPin_TypeDef DebugPin;

#endif
