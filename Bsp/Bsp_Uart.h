#ifndef __BSP_UART_H
#define __BSP_UART_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32h7xx_hal_uart.h"
#include "Bsp_GPIO.h"
#include "Bsp_DMA.h"

typedef void (*BspUART_Callback)(uint8_t *tx, uint16_t size);

typedef enum
{
    BspUART_CallbackType_Tx = 1,
    BspUART_CallbackType_Rx,
} BspUARTCallback_Type_List;

typedef enum
{
    BspUART_Port_4 = 0,
    BspUART_Port_6,
    BspUART_Port_7,
    BspUART_Port_Sum,
} BspUART_Port_List;

#pragma pack(1)
typedef struct
{
    uint16_t baudrate;
    bool pin_swap;

    USART_TypeDef *instance;
    UART_HandleTypeDef hdl;

    BspDMA_List tx_dma;
    BspDMA_Stream_List tx_stream;
    DMA_HandleTypeDef tx_dma_hdl;

    BspDMA_List rx_dma;
    BspDMA_Stream_List rx_stream;
    DMA_HandleTypeDef rx_dma_hdl;

    BspGPIO_Obj_TypeDef tx_io;
    BspGPIO_Obj_TypeDef rx_io;

    BspUART_Callback TxCallback;
    BspUART_Callback RxCallback;

    uint8_t *rx_buf;
    uint16_t rx_size;

    uint32_t cust_data_addr;
} BspUARTObj_TypeDef;
#pragma pack()

typedef struct
{
    bool (*init)(BspUARTObj_TypeDef *obj);
    bool (*de_init)(BspUARTObj_TypeDef *obj);
    bool (*set_baudrate)(BspUARTObj_TypeDef *obj, uint16_t baudrate);
    bool (*set_callback)(BspUARTObj_TypeDef *obj, BspUARTCallback_Type_List type, BspUART_Callback callback);
    BspUART_Callback (*get_callback)(BspUARTObj_TypeDef *obj, BspUARTCallback_Type_List type);
    bool (*send)(BspUARTObj_TypeDef *obj, uint8_t *tx, uint32_t size);
    bool (*get_send_state)(BspUARTObj_TypeDef *obj);
    bool (*receive)(BspUARTObj_TypeDef *obj, uint8_t *rx, uint16_t size);
    bool (*reset_receive)(BspUARTObj_TypeDef *obj);
    uint16_t (*get_RecCount)(BspUARTObj_TypeDef *obj);
} BspUART_TypeDef;

void UART_Idle_Callback(BspUART_Port_List index);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

extern BspUART_TypeDef BspUart;

#endif
