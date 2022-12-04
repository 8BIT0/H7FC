#ifndef __DEV_RECEIVER_H
#define __DEV_RECEIVER_H

#include "Dev_Sbus.h"
#include "Dev_CRSF.h"
#include "Bsp_GPIO.h"
#include "Bsp_Uart.h"

#define CHANNEL_RANGE_MIN 950
#define CHANNEL_RANGE_MAX 2050

typedef bool (*DevReceiver_Callback)(uint8_t *ptr, uint16_t size);

typedef enum
{
    Receiver_Type_Sbus = 0,
    Receiver_Type_CRSF,
} DevReceiver_TypeList;

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
} DevRecveiver_FunctionalDef_List;

#pragma pack(1)
typedef struct
{
    uint8_t channel_func_def[Receiver_Channel_Sum];
    uint16_t val_list[Receiver_Channel_Sum];
    uint32_t time_stamp;

    bool valid;
    bool failsafe;
} DevReceiverData_TypeDef;

typedef struct
{
    DevReceiver_TypeList Frame_type;

    uint16_t baudrate;
    uint32_t port_addr;

    uint8_t channel_num;

    DevReceiverData_TypeDef data;
    DevReceiver_Callback cb;

    void *port_ptr;

    /* for sbus receiver we gonna need inverter hardware */
    uint32_t invert_port;
    uint32_t invert_pin;

    bool (*inverter_init)(uint32_t port, uint32_t pin);
    bool (*invert_control)(uint32_t port, uint32_t pin);

    bool (*port_init)(uint8_t port_id, uint16_t baudrate);
} DevReceiverObj_TypeDef;
#pragma pack()

typedef struct
{
    bool (*init)(DevReceiverObj_TypeDef *obj, uint8_t channel_num);
    bool (*enable_control)(DevReceiverObj_TypeDef *obj, bool state);
    bool (*set_decode_callback)(DevReceiverObj_TypeDef *obj);
    bool (*conver_to)(DevReceiverObj_TypeDef *obj, DevReceiver_TypeList type, uint8_t *ptr, uint16_t size);
    DevReceiverData_TypeDef (*get)(DevReceiverObj_TypeDef *obj);
} DevReceiver_TypeDef;

#endif
