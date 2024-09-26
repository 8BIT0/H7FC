#ifndef __DEV_50HZPWM_H
#define __DEV_50HZPWM_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef struct
{

} Dev50HzPWMObj_TypeDef;

typedef struct
{
    bool (*init)();
    bool (*control)(uint16_t val);
} Dev50HzPWM_TypeDef;

#endif
