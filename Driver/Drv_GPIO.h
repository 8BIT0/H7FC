#ifndef __DRV_GPIO_H
#define __DRV_GPIO_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "Bsp_GPIO.h"

typedef void (*Exti_CallBack)(void);

typedef enum
{
    DrvGPIO_OutMode_Open = 0,
    DrvGPIO_ExtiMode_Open,
    DrvGPIO_InMode_Open,
} DrvGPIO_CMD_List;

#pragma pack(1)
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    bool level;
} DrvGPIOObj_TypeDef;
#pragma pack()

typedef struct
{
    void (*ctl)(DrvGPIO_CMD_List cmd, DrvGPIOObj_TypeDef Obj, void *arg);
    bool (*get)(DrvGPIOObj_TypeDef Obj);
    void (*set)(DrvGPIOObj_TypeDef Obj, bool state);
    void (*set_callback)(DrvGPIOObj_TypeDef Obj, Exti_CallBack func);
} DrvGPIO_TypeDef;

#endif
