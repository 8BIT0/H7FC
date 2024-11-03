#ifndef __SRV_LEDSTRIP_H
#define __SRV_LEDSTRIP_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

typedef struct
{
    bool (*init)(uint8_t led_num);
    void (*polling)(void);
} SrvLedStrip_TypeDef;

extern SrvLedStrip_TypeDef SrvLedStrip;

#endif
