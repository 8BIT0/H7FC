#ifndef __BSP_UART_H
#define __BSP_UART_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "Bsp_Uart_Port_Def.h"
#include "stm32h7xx_hal_uart.h"
#include "Bsp_GPIO.h"
#include "Bsp_DMA.h"

#define UART_HandleType_Size sizeof(UART_HandleTypeDef)
#define UART_DMA_Handle_Size sizeof(DMA_HandleTypeDef)

typedef enum
{
    BspUART_Port_1 = 0,
    BspUART_Port_4,
    BspUART_Port_6,
    BspUART_Port_7,
    BspUART_Port_Sum,
} BspUART_Port_List;

void UART_IRQ_Callback(BspUART_Port_List index);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
UART_HandleTypeDef *BspUart_GetObj_Handle(BspUART_Port_List index);

extern BspUART_TypeDef BspUart;

#endif
