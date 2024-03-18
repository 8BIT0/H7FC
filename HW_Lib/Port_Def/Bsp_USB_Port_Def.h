#ifndef __BSP_USB_PORT_DEF_H
#define __BSP_USB_PORT_DEF_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "CusQueue.h"

#define USB_VCP_MAX_TX_SIZE 512
#define USB_VCP_TX_BUFF_SIZE 1024

typedef void (*BspUSB_Tx_Cplt_Callback_Def)(uint32_t cus_data_addr, uint8_t *p_data, uint32_t *len);
typedef void (*BspUSB_Rx_Callback_Def)(uint32_t cus_data_addr, uint8_t *p_data, uint16_t len);
typedef void (*BspUSB_Connect_Callback_Def)(uint32_t cus_data_addr, uint32_t *sys_tick_val);

typedef enum
{
    BspUSB_None_Init = -1,
    BspUSB_Error_None = 0,
    BspUSB_Error_QueueCreate,
    BspUSB_Error_Init,
    BspUSB_Error_Semphr_Crt,
    BspUSB_Error_Busy,
    BspUSB_Error_EMEM,
    BspUSB_Error_Fail,
    BspUSB_Error_Unknow,
}BspUSB_Error_List;

typedef struct
{
    uint32_t tx_cnt;
    uint32_t tx_abort;
    uint32_t tx_fin_cnt;
    uint32_t tx_err_cnt;
}BspUSB_VCP_TxStatistic_TypeDef;

typedef struct
{
    uint8_t single_tx_buffer[USB_VCP_TX_BUFF_SIZE];

    BspUSB_Error_List init_state;
    QueueObj_TypeDef SendQueue;

    uint32_t rx_byte_sum;
    uint32_t tx_byte_sum;

    uint32_t rx_irq_cnt;
    uint32_t tx_cnt;
    uint32_t tx_abort_cnt;
    uint32_t tx_fin_cnt;
    uint32_t tx_err_cnt;
    uint32_t connect_time;

    BspUSB_Tx_Cplt_Callback_Def tx_fin_callback;
    BspUSB_Rx_Callback_Def rx_callback;
    BspUSB_Connect_Callback_Def connect_callback;

    uint32_t cus_data_addr;
}BspUSB_VCP_Obj_TypeDef;

typedef struct
{
    BspUSB_Error_List (*init)(uint32_t cus_data_addr);
    BspUSB_Error_List (*de_init)(void);
    BspUSB_Error_List (*send)(uint8_t *p_data, uint16_t len);
    void (*set_rx_callback)(BspUSB_Rx_Callback_Def callback);
    void (*set_tx_cpl_callback)(BspUSB_Tx_Cplt_Callback_Def callback);
    void (*set_connect_callback)(BspUSB_Connect_Callback_Def callback);
    BspUSB_VCP_TxStatistic_TypeDef (*get_tx_statistic)(void);
    bool (*check_connect)(uint32_t sys_tick, uint32_t time_out);
}BspUSB_VCP_TypeDef;

#endif
