#ifndef __BSP_USB_H
#define __BSP_USB_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_desc.h"
#include "CusQueue.h"

typedef void (*BspUSB_Tx_Cplt_Callback_Def)(uint8_t *p_data, uint32_t *len);
typedef void (*BspUSB_Rx_Callback_Def)(uint8_t *p_data, uint16_t len);

typedef enum
{
    BspUSB_None_Init = -1,
    BspUSB_Error_None = 0,
    BspUSB_Error_QueueCreate,
    BspUSB_Error_Init,
    BspUSB_Error_Busy,
    BspUSB_Error_EMEM,
    BspUSB_Error_Fail,
    BspUSB_Error_Unknow,
}BspUSB_Error_List;

typedef struct
{
    BspUSB_Error_List init_state;
    QueueObj_TypeDef SendQueue;

    uint32_t rx_byte_sum;
    uint32_t tx_byte_sum;

    uint32_t rx_irq_cnt;
    uint32_t tx_cnt;
    uint32_t tx_queue_reset_cnt;
    uint32_t tx_fin_cnt;
    uint32_t tx_err_cnt;

    BspUSB_Tx_Cplt_Callback_Def tx_fin_callback;
    BspUSB_Rx_Callback_Def rx_callback;
}BspUSB_VCP_Obj_TypeDef;

typedef struct
{
    BspUSB_Error_List (*init)(void);
    BspUSB_Error_List (*send)(uint8_t *p_data, uint16_t len);
    void (*set_rx_callback)(BspUSB_Rx_Callback_Def callback);
    void (*set_tx_cpl_callback)(BspUSB_Tx_Cplt_Callback_Def callback);
}BspUSB_VCP_TypeDef;

extern BspUSB_VCP_TypeDef BspUSB_VCP;

#endif
