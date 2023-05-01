#ifndef __SRV_PROTOCOL_H
#define __SRV_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "mavlink.h"

typedef enum
{
    SrvProto_Port_Serial,
    SrvProto_Port_USB,
    SrvProto_Port_ETH,
} SrvProto_Port_TypeList;

typedef struct
{
    SrvProto_Port_TypeList port_type;

    void *obj;
    void *obj_cfg;
    void *api;

    bool is_active;
    bool in_using;

    uint8_t *p_buf;
    uint32_t p_buf_size;
} SrvProto_PortObj_TypeDef;

#endif
