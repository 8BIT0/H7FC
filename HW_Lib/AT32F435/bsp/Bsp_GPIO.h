#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "at32f435_437.h"
#include "Bsp_GPIO_Port_Def.h"

#define GPIO_EXTI_SUM 16

#define To_GPIO_Port(x) ((BspGPIO_Port_TypeDef *)x)
#define To_ExtiGPIO_Port(x) ((BspGPIO_EXTI_Port_TypeDef *)x)

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

#ifdef __cplusplus
}
#endif

#endif
