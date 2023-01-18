#ifndef __FRAME_H
#define __FRAME_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define FRAEM_HEADER_1 0xFF
#define FRAEM_HEADER_2 0xFE

#define FRAME_HEADER_DIR 0
#define FRAME_HEARTBEAT_SIZE 0
#define FRAME_HEARTBEAT_ENDER 0xFEFF

#define FRAME_HEARTBEAT_TIMEOUT 200 // unit: ms

typedef bool (*frame_decode_callback)(uint8_t *data_in);

typedef enum
{
    Frame_Decode_NoneError = 0,
    Frame_Decode_Header_Error,
    Frame_Decode_Type_Error,
    Frame_Decode_Dir_Error,
    Frame_Decode_RecData_Error,
    Frame_Decode_HeartBeat_Error,
} Frame_Decode_ErrorCode_List;

typedef enum
{
    Frame_Type_HeartBeat = 0,
    Frame_Type_Receiver,
    Frame_Type_IMU,
} Frame_TypeList;

typedef enum
{
    Frame_ReceiverData_Out = 0,
    Frame_ReceiverChannel_Set = 1,
} Frame_Receiver_DirList;

typedef struct
{
    uint32_t update_rt;

    uint32_t err_cnt;
    uint32_t receiver_setting_err_cnt;

    uint32_t receiver_setting_cnt;
} Frame_Monitor_TypeDef;

#pragma pack(1)
typedef struct
{
    uint8_t header_1;
    uint8_t header_2;
    uint8_t type;
    uint8_t dir;
    uint8_t size;
} Frame_Format_TypeDef;

typedef struct
{
    uint16_t crc16;
} Frame_Receiver_TypeDef; /* receiver data output */

typedef struct
{
    uint16_t crc16;
} Frame_IMU_TypeDef;

typedef struct
{
    uint16_t crc16;
} Frame_INS_TypeDef;

typedef struct
{
    uint16_t crc16;
} Frame_MotoOut_TypeDef;

typedef struct
{
    uint16_t crc16;
} Frame_BaseSetting_TypeDef;

typedef struct
{
    uint16_t crc16;
} Frame_ControlParam_TypeDef;

typedef struct
{
    uint16_t crc16;
} Frame_SerialPort_TypeDef;

typedef struct
{
    uint16_t crc16;
} Frame_ChannelSetting_TypeDef;
#pragma pack()

Frame_Decode_ErrorCode_List Frame_Decode(uint8_t *p_data, uint16_t size);

#endif
