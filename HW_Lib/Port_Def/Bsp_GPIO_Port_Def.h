#ifndef __BSP_GPIO_PORT_DEF_H
#define __BSP_GPIO_PORT_DEF_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef void (*EXTI_Callback)(void);

typedef enum
{
    GPIO_Exti_Rasing = 1,
    GPIO_Exti_Falling,
    GPIO_Exti_TwoEdge,
} BspGPOP_ExtiMode_List;

typedef struct
{
    void *port;
    uint16_t pin;
    uint8_t init_state;
    uint32_t alternate;
} BspGPIO_Obj_TypeDef;

typedef struct
{
    bool (*exti_init)(BspGPIO_Obj_TypeDef IO_Obj, EXTI_Callback callback);
    bool (*out_init)(BspGPIO_Obj_TypeDef IO_Obj);
    bool (*in_init)(BspGPIO_Obj_TypeDef IO_Obj);
    bool (*alt_init)(BspGPIO_Obj_TypeDef IO_Obj, uint32_t af_mode);
    bool (*read)(BspGPIO_Obj_TypeDef IO_Obj);
    bool (*set_exti_callback)(BspGPIO_Obj_TypeDef IO_Obj, EXTI_Callback callback);
    bool (*set_exti_mode)(BspGPIO_Obj_TypeDef IO_Obj, BspGPOP_ExtiMode_List mode);
    void (*write)(BspGPIO_Obj_TypeDef IO_Obj, bool state);
    void (*de_init)(BspGPIO_Obj_TypeDef IO_Obj);
} BspGPIO_TypeDef;

#endif
