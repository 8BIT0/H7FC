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

typedef enum
{
    Proto_IMU = 0,
    Proto_INS,
    Proto_Filter,
    Proto_Control,
    Proto_Remote,
    Proto_Config,
} ProtType_List;

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
