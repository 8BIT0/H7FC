#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "at32f435_437.h"
#include "Bsp_GPIO_Port_Def.h"

#define GPIO_EXTI_SUM 16

typedef struct
{
    gpio_type *port;
} BspGPIO_Port_TypeDef;

typedef struct
{
    scfg_port_source_type port;
} BspGPIO_EXTI_Port_TypeDef;

extern BspGPIO_TypeDef BspGPIO;

void BspGPIO_IRQ_Polling(uint32_t exti_line);

#endif
