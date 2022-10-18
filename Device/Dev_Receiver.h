#ifndef __DEV_RECEIVER_H
#define __DEV_RECEIVER_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define RECEIVER_MAX_CHANNEL 32

typedef bool (*DevReceiver_Callback)(uint8_t *ptr, uint16_t size);

typedef enum
{
    Receiver_Type_Sbus = 0,
    Receiver_Type_CRSF,
} DevReceiver_TypeList;

typedef enum
{
    Receiver_In = 0,
    Receiver_Out,
} DecReceiver_IO_TypeList;

#pragma pack(1)
typedef union {
    uint16_t val_list[RECEIVER_MAX_CHANNEL];

    struct
    {
        uint16_t pitch : 16;
        uint16_t roll : 16;
        uint16_t throttle : 16;
        uint16_t yaw : 16;

        uint16_t aux_1 : 16;
        uint16_t aux_2 : 16;
        uint16_t aux_3 : 16;
        uint16_t aux_4 : 16;

        uint16_t aux_5 : 16;
        uint16_t aux_6 : 16;
        uint16_t aux_7 : 16;
        uint16_t aux_8 : 16;

        uint16_t aux_9 : 16;
        uint16_t aux_10 : 16;
        uint16_t aux_11 : 16;
        uint16_t aux_12 : 16;

        uint16_t aux_13 : 16;
        uint16_t aux_14 : 16;
        uint16_t aux_15 : 16;
        uint16_t aux_16 : 16;

        uint16_t aux_17 : 16;
        uint16_t aux_18 : 16;
        uint16_t aux_19 : 16;
        uint16_t aux_20 : 16;

        uint16_t aux_21 : 16;
        uint16_t aux_22 : 16;
        uint16_t aux_23 : 16;
        uint16_t aux_24 : 16;

        uint16_t aux_25 : 16;
        uint16_t aux_26 : 16;
        uint16_t aux_27 : 16;
        uint16_t aux_28 : 16;
    } channel;
} DevReceiverDecodeData_TypeDef;

typedef struct
{
    DevReceiverDecodeData_TypeDef val;
    uint32_t time_stamp;
} DevReceiverData_TypeDef;

typedef struct
{
    DevReceiver_TypeList Frame_type;
    DecReceiver_IO_TypeList IO_type;

    uint8_t port_id;
    uint16_t baudrate;

    uint8_t channel_num;

    DevReceiverData_TypeDef data;
    DevReceiver_Callback cb;
} DevReceiverObj_TypeDef;
#pragma pack()

typedef struct
{
    /* if IO_type is receiver data out then need source data structure to copy and convert receiver value */
    /* else src set NULL */
    bool (*init)(DevReceiverObj_TypeDef *obj, DecReceiver_IO_TypeList IO_type, DevReceiverObj_TypeDef *src);
    bool (*set_decode_callback)(DevReceiverObj_TypeDef cb);
    DevReceiverData_TypeDef (*get)(DevReceiverObj_TypeDef *obj);
} DevReceiver_TypeDef;

#endif
