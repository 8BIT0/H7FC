#ifndef __BSP_UART_H
#define __BSP_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Bsp_Uart_Port_Def.h"

typedef enum
{
    Bsp_UART_Port_None = 0,
    Bsp_UART_Port_1,
    Bsp_UART_Port_2,
    Bsp_UART_Port_3,
    Bsp_UART_Port_4,
    Bsp_UART_Port_5,
    Bsp_UART_Port_6,
    Bsp_UART_Port_7,
    Bsp_UART_Port_8,
    Bsp_UART_Port_Sum,
} BspUART_Port_List;

void BspUart_Irq_Callback(void *arg);

extern BspUART_TypeDef BspUart;

#ifdef __cplusplus
}
#endif

#endif

