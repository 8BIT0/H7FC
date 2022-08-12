#ifndef __TASK_PROTOCOL_H
#define __TASK_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "scheduler.h"
#include "runtime.h"

typedef enum
{
    ProtoQueue_Idle = 0,
    ProtoQueue_Ok,
    ProtoQueue_Busy,
    ProtoQueue_Full,
    ProtoQueeu_Error,
} ProtoQueue_State_List;

#pragma pack(1)
typedef struct
{
    int16_t Org_Acc[3];
    int16_t Org_Gyr[3];

    float Flt_Acc[3];
    float Flt_Gyr[3];

    int16_t Org_temp;

    SYSTEM_RunTime Rt;
} ProtIMUData_TypeDef;
#pragma pack()

typedef enum
{
    TaskProto_Init = 0,
    TaskProto_Core,
    TaskProto_Error_Proc,
} TaskProto_State_List;

bool TaskProtocol_Init(void);
void TaskProtocol_Core(Task_Handle hdl);
ProtoQueue_State_List TaskProto_PushProtocolQueue(uint8_t *p_data, uint16_t size);

#endif
