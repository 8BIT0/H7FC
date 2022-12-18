#ifndef __TASK_TELEMETRY_H
#define __TASK_TELEMETRY_H

#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "../System/scheduler/scheduler.h"
#include "Srv_Receiver.h"

#define Telemetry_SetBit(x) (1 << x)

#define TELEMETRY_SET_ARM true
#define TELEMETRY_SET_DISARM false
#define TELEMETRY_DISABLE_ALL_MODULE 0

#pragma pack(1)
typedef enum
{
    Telemetry_Control_Mode_ACRO = 1,
    Telemetry_Control_Mode_STAB,
    Telemetry_Control_Mode_OSD_Tune,
    Telemetry_Control_Mode_Default = Telemetry_Control_Mode_STAB,
} Telemetry_ControlMode_Type;

typedef enum
{
    Telemetry_Control_HoldAlt = Telemetry_SetBit(0),
    Telemetry_Control_HeadingLock = Telemetry_SetBit(1),
    Telemetry_Control_AngleLimit = Telemetry_SetBit(2),
    Telemetry_Control_Buzzer = Telemetry_SetBit(3),
    Telemetry_Control_Log = Telemetry_SetBit(4),
} Telemetry_ModuleControl_List;

typedef struct
{
    bool init_state;
    bool arm_state;
    uint8_t control_mode;
    uint16_t module_enable;
} Telemetry_RCInput_TypeDef;
#pragma pack()

void TaskTelemetry_Init(void);
void TaskTelemetry_Core(Task_Handle hdl);

#endif
