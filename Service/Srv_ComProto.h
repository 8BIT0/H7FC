#ifndef __SRV_COMPROTO_H
#define __SRV_COMPROTO_H

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

typedef bool (*ComProto_Callback)(uint8_t *p_data, uint32_t len);

typedef enum
{
    SrvComProto_Type_Cus = 0,
    SrvComProto_Type_MAV,
} SrvComProto_Type_List;

typedef enum
{

} SrvComProto_MAVMsg_List;

typedef union
{
    struct section
    {
        uint8_t msg_actuator_control_target : 1;
    };

    uint64_t val[4];
} SrvComProto_MavReg_TypeDef;

typedef struct
{
    SrvComProto_Type_List Proto_Type;
    SrvComProto_MavReg_TypeDef MavSet_Reg;

    uint8_t *proto_buf;
    uint16_t proto_buf_size;
} SrvComProto_Monitor_TypeDef;

#endif
