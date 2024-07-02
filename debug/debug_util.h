#ifndef __DEBUG_UTIL_H
#define __DEBUG_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include "Bsp_GPIO.h"
#include "Bsp_Uart.h"

void assert(bool state);

#pragma pack(1)
typedef BspGPIO_Obj_TypeDef DebugPinObj_TypeDef;
#pragma pack()

typedef struct
{
    void *port_obj;
    bool init;
    uint8_t *p_buf;
    void* (*malloc)(uint32_t size);
    void (*free)(void *ptr);
} DebugPrintObj_TypeDef;

typedef struct
{
    bool (*init)(DebugPinObj_TypeDef pin);
    bool (*ctl)(DebugPinObj_TypeDef pin, bool state);
} DebugPin_TypeDef;

void Debug_Port_Init(DebugPrintObj_TypeDef *Obj);
void Debug_Print(DebugPrintObj_TypeDef *Obj, const char *tag, const char *fmt, ...);
extern DebugPin_TypeDef DebugPin;

#ifdef __cplusplus
}
#endif

#endif
