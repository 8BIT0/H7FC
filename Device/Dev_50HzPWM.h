#ifndef __DEV_50HZPWM_H
#define __DEV_50HZPWM_H

#ifdef __cplusplus
extern "C" {
#endif

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

#ifdef __cplusplus
}
#endif

#endif
