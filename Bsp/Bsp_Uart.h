#ifndef __BSP_UART_H
#define __BSP_UART_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#pragma pack(1)
typedef struct
{
    uint8_t id;
} BspUARTObj_TypeDef;
#pragma pack()

typedef struct
{
    bool (*init)(BspUARTObj_TypeDef *obj);
    bool (*write)(BspUARTObj_TypeDef *obj, uint8_t *tx, uint32_t size);
} BspUART_TypeDef;

#endif
