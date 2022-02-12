#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_gpio.h"

#pragma pack(1)
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_InitTypeDef cfg_structure;
    GPIO_PinState init_state;
} BspGPIO_Obj_TypeDef;
#pragma pack()

typedef struct
{
    bool (*init)(BspGPIO_Obj_TypeDef IO_Obj);
    bool (*read)(uint32_t port, uint16_t pin);
    void (*write)(uint32_t port, uint16_t pin, bool state);
} BspGPIO_TypeDef;

extern BspGPIO_TypeDef BspGPIO;

#endif
