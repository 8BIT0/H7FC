#ifndef __BSP_UART_H
#define __BSP_UART_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef void (*UartTx_Callback)(uint8_t *tx, uint16_t size);
typedef void (*UartRx_Callback)(uint8_t *rx, uint16_t size);

#pragma pack(1)
typedef struct
{
    uint8_t id;

    UartTx_Callback TxCallback;
    UartRx_Callback RxCallback;

    uint8_t *TxBuff;
    uint8_t *RxBuff;

    uint16_t TxSize;
    uint16_t RxSize;
} BspUARTObj_TypeDef;
#pragma pack()

typedef struct
{
    bool (*init)(BspUARTObj_TypeDef *obj);
    bool (*de_init)(BspUARTObj_TypeDef *obj);
    bool (*write)(BspUARTObj_TypeDef *obj, uint8_t *tx, uint32_t size);
    uint16_t (*get_RecCount)(BspUARTObj_TypeDef *obj);
} BspUART_TypeDef;

#endif
