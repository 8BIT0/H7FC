#ifndef __DEV_LED_H
#define __DEV_LED_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

typedef struct
{
    uint32_t port;
    uint16_t pin;
    bool default_state;
} DevLedObj_TypeDef;

typedef struct
{
    bool (*init)(DevLedObj_TypeDef obj);
    bool (*ctl)(DevLedObj_TypeDef obj, bool state);
} DevLed_TypeDef;

extern DevLed_TypeDef DevLED;

#endif
