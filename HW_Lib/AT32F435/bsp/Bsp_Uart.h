#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "Bsp_Uart_Port_Def.h"

typedef enum
{
    BspUART_Port_1 = 0,
    BspUART_Port_2,
    BspUART_Port_3,
    BspUART_Port_4,
    BspUART_Port_5,
    BspUART_Port_6,
    BspUART_Port_7,
    BspUART_Port_8,
    BspUART_Port_Sum,
} BspUART_Port_List;

void BspUart_Irq_Callback(void *arg);

extern BspUART_TypeDef BspUart;

#endif

