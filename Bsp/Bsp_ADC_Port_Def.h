#ifndef __BSP_ADC_PORT_DEF_H
#define __BSP_ADC_PORT_DEF_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>

typedef struct
{
    uint32_t Port;
    uint32_t Pin;
}BspADCObj_TypeDef;

typedef struct
{
    bool (*init)(const BspADCObj_TypeDef obj);
    uint32_t (*get)(const BspADCObj_TypeDef obj);
}BspADC_TypeDef;

#endif

