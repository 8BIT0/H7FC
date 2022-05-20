#ifndef __TASK_PROTOCOL_H
#define __TASK_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "scheduler.h"
#include "runtime.h"

#pragma pack(1)
typedef struct
{
    int16_t Org_Acc[3];
    int16_t Org_Gyr[3];

    double Flt_Acc[3];
    double Flt_Gyr[3];

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

#endif
