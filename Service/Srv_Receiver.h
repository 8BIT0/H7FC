#ifndef __SRV_RECEIVER_H
#define __SRV_RECEIVER_H

#include "Dev_Sbus.h"
#include "Dev_CRSF.h"
#include "Bsp_GPIO.h"
#include "Bsp_Uart.h"

#define CHANNEL_RANGE_MIN 950
#define CHANNEL_RANGE_MAX 2050

typedef bool (*SrvReceiver_Callback)(uint8_t *ptr, uint16_t size);

typedef enum
{
    Receiver_Type_Sbus = 0,
    Receiver_Type_CRSF,
} SRvReceiver_TypeList;

typedef enum
{
    Receiver_ChannelID_Pitch = 0,
    Receiver_ChannelID_Roll,
    Receiver_ChannelID_Throttle,
    Receiver_ChannelID_Yaw,
    Receiver_ChannelID_AUX_1,
    Receiver_ChannelID_AUX_2,
    Receiver_ChannelID_AUX_3,
    Receiver_ChannelID_AUX_4,
    Receiver_ChannelID_AUX_5,
    Receiver_ChannelID_AUX_6,
    Receiver_ChannelID_AUX_7,
    Receiver_ChannelID_AUX_8,
    Receiver_ChannelID_AUX_9,
    Receiver_ChannelID_AUX_10,
    Receiver_ChannelID_AUX_11,
    Receiver_ChannelID_AUX_12,

    Receiver_Channel_Sum,
} SrvRecveiver_FunctionalDef_List;

#pragma pack(1)
typedef struct
{
    uint8_t channel_func_def[Receiver_Channel_Sum];
    uint16_t val_list[Receiver_Channel_Sum];
    uint32_t time_stamp;

    bool valid;
    bool failsafe;
} SrvReceiverData_TypeDef;

typedef struct
{
    SrvReceiver_TypeList Frame_type;

    uint16_t baudrate;
    uint32_t port_addr;

    uint8_t channel_num;

    SrvReceiverData_TypeDef data;
    SrvReceiver_Callback cb;

    void *port_ptr;

    /* for sbus receiver we gonna need inverter hardware */
    uint32_t invert_port;
    uint32_t invert_pin;

    bool (*inverter_init)(uint32_t port, uint32_t pin);
    bool (*invert_control)(uint32_t port, uint32_t pin);

    bool (*port_init)(uint8_t port_id, uint16_t baudrate);
} SrvReceiverObj_TypeDef;
#pragma pack()

typedef struct
{
    bool (*init)(SrvReceiverObj_TypeDef *obj, uint8_t channel_num);
    bool (*enable_control)(SrvReceiverObj_TypeDef *obj, bool state);
    bool (*set_decode_callback)(SrvReceiverObj_TypeDef *obj);
    bool (*conver_to)(SrvReceiverObj_TypeDef *obj, SrvReceiver_TypeList type, uint8_t *ptr, uint16_t size);
    SrvReceiverData_TypeDef (*get)(SrvReceiverObj_TypeDef *obj);
} SrvReceiver_TypeDef;

#endif
