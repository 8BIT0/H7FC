#ifndef __SRV_RECEIVER_H
#define __SRV_RECEIVER_H

#include "runtime.h"
#include "Dev_Sbus.h"
#include "Dev_CRSF.h"
#include "Bsp_GPIO.h"
#include "Bsp_Uart.h"

#define SRV_RECEIVER_UPDATE_TIMEOUT_MS 100
#define SRV_RECEIVER_BUFF_SIZE 256

#define CHANNEL_RANGE_MIN 100
#define CHANNEL_RANGE_MID 1000
#define CHANNEL_RANGE_MAX 1900

typedef enum
{
    Receiver_Port_Serial = 1,
    Receiver_Port_Spi,
} SrvReceiver_Port_TypeList;

typedef enum
{
    Receiver_Type_Sbus = 0,
    Receiver_Type_CRSF,
    Receiver_Type_SPI_Cus,
} SrvReceiver_Frame_TypeList;

typedef enum
{
    Receiver_No_Error = 0,
    Receiver_Obj_Error,
    Receiver_Port_Error,
    Receiver_Port_Init_Error,
    Receiver_FrameType_Error,
} SrvReceiver_ErrorCode_List;

#pragma pack(1)
typedef struct
{
    bool FailSafe : 1;
    bool Update_TimeOut : 1;
    SrvReceiver_Port_TypeList PortType : 2;
    SrvReceiver_Frame_TypeList FrameType : 2;

    uint8_t init_error_code;

    uint32_t cur_rt;
    uint32_t lst_update_rt;

    uint32_t success_decode_cnt;
    uint32_t error_decode_cnt;
    uint32_t total_decode_cnt;

    uint32_t over_rang_cnt;
    uint32_t value_step_bump_cnt;
} SrvReceiver_Monitor_TypeDef;

#define SRVRECEIVER_SIZE sizeof(SrvReceiver_Monitor_TypeDef)

typedef struct
{
    uint32_t time_stamp;

    uint16_t *val_list;
    uint16_t rssi;
    uint16_t link_quality;

    bool failsafe;
} SrvReceiverData_TypeDef;

typedef struct
{
    void *api;
    void *cfg;
} SrvReceiver_Port_TypeDef;

typedef struct
{
    bool arm_state;
    bool buzz_state;
    bool osd_tune_state;
    bool failsafe;

    uint16_t gimbal_val[4];
    uint8_t control_mode;
    uint16_t module_enable;

    uint32_t update_interval;
    uint32_t time_stamp;

    uint8_t channel_sum;
    uint16_t channel[32];
} SrvRecever_RCSig_TypeDef;

typedef struct
{
    SrvReceiver_Frame_TypeList Frame_type;
    SrvReceiver_Port_TypeList port_type;
    void *port_cfg;
    uint16_t update_period;
    uint16_t baudrate;
    uint32_t port_addr;
    uint8_t channel_num;
    uint8_t *frame_data_obj;
    SrvReceiverData_TypeDef data;
    void *frame_api;
    SrvReceiver_Port_TypeDef *port;

    bool re_update;
    bool in_use;
    uint32_t invert_list;

    SYSTEM_RunTime OSDTune_TriggerMs;

    /* for sbus receiver we gonna need inverter hardware */
    BspGPIO_Obj_TypeDef inverter_pin;

    bool (*inverter_init)(BspGPIO_Obj_TypeDef pin);
    bool (*invert_control)(BspGPIO_Obj_TypeDef pin, bool state);
} SrvReceiverObj_TypeDef;
#pragma pack()

typedef struct
{
    uint8_t *(*create_serial_obj)(uint32_t serial_instance,
                                  uint32_t rx_dma,
                                  uint32_t rx_dma_stream,
                                  uint32_t tx_dma,
                                  uint32_t tx_dma_stream,
                                  bool swap,
                                  const BspGPIO_Obj_TypeDef tx_pin,
                                  const BspGPIO_Obj_TypeDef rx_pin);
    uint8_t *(*create_spi_obj)(void);
    bool (*init)(SrvReceiverObj_TypeDef *obj, uint8_t *port_obj);
    SrvReceiverData_TypeDef (*get)(SrvReceiverObj_TypeDef *obj);
    void (*invert)(SrvReceiverObj_TypeDef *obj, uint16_t channel_index);
} SrvReceiver_TypeDef;

extern SrvReceiver_TypeDef SrvReceiver;

#endif
