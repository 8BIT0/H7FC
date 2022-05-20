#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_gpio.h"

#define GPIO_EXTI_SUM 16
typedef void (*EXTI_Callback)(void);

typedef enum
{
    GPIO_Exti_Rasing = 1,
    GPIO_Exti_Falling,
    GPIO_Exti_TwoEdge,
}BspGPOP_ExtiMode_List;

#pragma pack(1)
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState init_state;
} BspGPIO_Obj_TypeDef;
#pragma pack()

typedef struct
{
    bool (*exti_init)(BspGPIO_Obj_TypeDef IO_Obj, EXTI_Callback callback);
    bool (*out_init)(BspGPIO_Obj_TypeDef IO_Obj);
    bool (*read)(uint32_t port, uint16_t pin);
    bool (*set_exti_callback)(BspGPIO_Obj_TypeDef IO_Obj, EXTI_Callback callback);
    bool (*set_exti_mode)(BspGPIO_Obj_TypeDef IO_Obj, BspGPOP_ExtiMode_List mode);
    void (*write)(uint32_t port, uint16_t pin, bool state);
} BspGPIO_TypeDef;

extern BspGPIO_TypeDef BspGPIO;

#endif
