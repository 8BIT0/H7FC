#ifndef __SRV_COMPROTO_H
#define __SRV_COMPROTO_H

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

typedef bool (*ComProto_Callback)(uint8_t *p_data, uint32_t len);

typedef union
{
    struct section
    {
    };

    uint64_t val;
} SrvComProto_MAVReg1_TypeDef;

typedef union
{
    struct section
    {
    };

    uint64_t val;
} SrvComProto_MavReg2_TypeDef;

typedef union
{
    struct section
    {
    };

    uint64_t val;
} SrvComProto_MavReg3_TypeDef;

typedef union
{
    struct section
    {
    };

    uint64_t val;
} SrvComProto_MavReg4_TypeDef;

typedef struct
{
    bool enable_mavlink;

    uint8_t *proto_buf;
    uint16_t proto_buf_size;

} SrvComProto_Monitor_TypeDef;

#endif
