#ifndef __BSP_UART_H
#define __BSP_UART_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32h7xx_hal_uart.h"
#include "Bsp_GPIO.h"
#include "Bsp_DMA.h"

typedef void (*BspUART_Callback)(uint8_t *cust_data_addr, uint8_t *buff, uint16_t size);

#define BspUart_Clock_Error -1
#define Bspuart_None_Index -1

typedef enum
{
    BspUART_CallbackType_Tx = 1,
    BspUART_CallbackType_Rx,
} BspUARTCallback_Type_List;

typedef enum
{
    BspUart_IRQ_Type_None = 0,
    BspUart_IRQ_Type_Idle,
    BspUart_IRQ_Type_Byte,
} BsuUart_IRQ_Type_List;

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
    uint32_t rx_cnt;
    uint32_t tx_cnt;

    uint32_t rx_err_cnt;
    uint32_t rx_full_cnt;

    uint32_t tx_success_cnt;
    uint32_t tx_err_cnt;
    uint32_t tx_unknow_err_cnt;
} BspUart_MonitorObj_TypeDef;

typedef struct
{
    uint32_t baudrate;
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

    uint8_t rx_single_byte;

    uint32_t cust_data_addr;

    bool wait_till_send_finish;
    bool send_finish;

    bool init_state;

    BsuUart_IRQ_Type_List irq_type;

    BspUart_MonitorObj_TypeDef monitor;
} BspUARTObj_TypeDef;
#pragma pack()

typedef struct
{
    bool (*init)(BspUARTObj_TypeDef *obj);
    bool (*set_parity)(BspUARTObj_TypeDef *obj, uint32_t parity);
    bool (*set_stop_bit)(BspUARTObj_TypeDef *obj, uint32_t stop_bit);
    bool (*set_data_bit)(BspUARTObj_TypeDef *obj, uint32_t data_bit);
    bool (*set_swap)(BspUARTObj_TypeDef *obj, bool swap);
    bool (*send)(BspUARTObj_TypeDef *obj, uint8_t *tx, uint32_t size);
} BspUART_TypeDef;

void UART_Idle_Callback(BspUART_Port_List index);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
UART_HandleTypeDef *BspUart_GetObj_Handle(BspUART_Port_List index);

extern BspUART_TypeDef BspUart;

#endif
