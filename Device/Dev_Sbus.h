#ifndef __DEV_SBUS_H
#define __DEV_SBUS_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>

#define SBUS_BAUDRATE 100000
#define SBUS_FRAME_BYTE_SIZE 25
#define SBUS_FRAME_HEADER 0xF0
#define SBUS_DECODE_MASK 0x07FF
#define SBUS_MAX_CHANNEL 16

typedef enum
{
    DevSBUS_NoError = 0,
    DevSBUS_Error_Obj,
    DevSBUS_Error_Frame,
} DevSBUS_ErrorCode_List;

#pragma pack(1)
typedef struct
{
    uint8_t channel;
    uint16_t val[SBUS_MAX_CHANNEL];
    bool valid;
} DevSBUSData_TypeDef;

typedef union
{
    struct
    {
        uint8_t ch17 : 1;
        uint8_t ch18 : 1;
        uint8_t frame_lost : 1;
        uint8_t fail_safe_act : 1;
        uint8_t reserve : 4;
    } bit;

    uint8_t val;
} DevSBUS_FuncBit_TypeDef;
#pragma pack()

typedef struct
{
    DevSBUS_ErrorCode_List (*decode)(uint8_t *ptr, uint16_t size, DevSBUSData_TypeDef *obj);
    DevSBUS_ErrorCode_List (*encode)(uint8_t *ptr, uint16_t size, const DevSBUSData_TypeDef obj);
} DevSBUS_TypeDef;

extern DevSBUS_TypeDef DevSBUS;

#endif
