#ifndef __BSP_UART_PORT_DEF_H
#define __BSP_UART_PORT_DEF_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "Bsp_GPIO_Port_Def.h"

typedef void (*BspUART_Callback)(uint32_t cust_data_addr, uint8_t *buff, uint16_t size);

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
} BspUart_IRQ_Type_List;

typedef struct
{
    uint32_t rx_cnt;
    uint32_t tx_cnt;

    uint32_t rx_err_cnt;
    uint32_t ore_cnt;
    uint32_t rx_full_cnt;

    uint32_t tx_success_cnt;
    uint32_t tx_err_cnt;
    uint32_t tx_unknow_err_cnt;
} BspUart_MonitorObj_TypeDef;

typedef struct
{
    uint32_t baudrate;
    bool pin_swap;

    void *instance;
    void *hdl;

    int8_t tx_dma;
    int8_t tx_stream;
    void *tx_dma_hdl;

    int8_t rx_dma;
    int8_t rx_stream;
    void *rx_dma_hdl;

    BspGPIO_Obj_TypeDef tx_io;
    BspGPIO_Obj_TypeDef rx_io;

    BspUART_Callback TxCallback;
    BspUART_Callback RxCallback;

    uint8_t *rx_buf;
    uint16_t rx_size;

    uint8_t rx_single_byte;

    uint32_t cust_data_addr;

    bool init_state;

    BspUart_IRQ_Type_List irq_type;

    BspUart_MonitorObj_TypeDef monitor;
} BspUARTObj_TypeDef;

typedef struct
{
    bool (*init)(BspUARTObj_TypeDef *obj);
    bool (*de_init)(BspUARTObj_TypeDef *obj);
    bool (*set_parity)(BspUARTObj_TypeDef *obj, uint32_t parity);
    bool (*set_stop_bit)(BspUARTObj_TypeDef *obj, uint32_t stop_bit);
    bool (*set_data_bit)(BspUARTObj_TypeDef *obj, uint32_t data_bit);
    bool (*set_swap)(BspUARTObj_TypeDef *obj, bool swap);
    bool (*send)(BspUARTObj_TypeDef *obj, uint8_t *tx, uint16_t size);
    bool (*set_rx_callback)(BspUARTObj_TypeDef *obj, BspUART_Callback callback);
    bool (*set_tx_callback)(BspUARTObj_TypeDef *obj, BspUART_Callback callback);
} BspUART_TypeDef;

#endif
