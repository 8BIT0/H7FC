#ifndef __TASK_TELEMETRY_H
#define __TASK_TELEMETRY_H

#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "../System/scheduler/scheduler.h"
#include "Srv_Receiver.h"
#include "linked_list.h"

#define Telemetry_SetBit(x) (1 << x)

#define TELEMETRY_SET_ARM true
#define TELEMETRY_SET_DISARM false
#define TELEMETRY_DISABLE_ALL_MODULE 0

#define TELEMETRY_RC_CHANNEL_MIN_VAL CHANNEL_RANGE_MIN
#define TELEMETRY_RC_CHANNEL_MAX_VAL CHANNEL_RANGE_MAX

#pragma pack(1)
typedef enum
{
    Telemetry_RC_Pitch = Telemetry_SetBit(0),
    Telemetry_RC_Roll = Telemetry_SetBit(1),
    Telemetry_RC_Throttle = Telemetry_SetBit(2),
    Telemetry_RC_Yaw = Telemetry_SetBit(3),
    Telemetry_Gimbal_TagSum = 4,
    Telemetry_RC_AUX_1 = Telemetry_SetBit(4),
    Telemetry_RC_AUX_2 = Telemetry_SetBit(5),
    Telemetry_RC_AUX_3 = Telemetry_SetBit(6),
    Telemetry_RC_AUX_4 = Telemetry_SetBit(7),
    Telemetry_RC_AUX_5 = Telemetry_SetBit(8),
    Telemetry_RC_AUX_6 = Telemetry_SetBit(9),
    Telemetry_RC_AUX_7 = Telemetry_SetBit(10),
    Telemetry_RC_AUX_8 = Telemetry_SetBit(11),
    Telemetry_RC_AUX_9 = Telemetry_SetBit(12),
    Telemetry_RC_AUX_10 = Telemetry_SetBit(13),
    Telemetry_RC_AUX_11 = Telemetry_SetBit(14),
    Telemetry_RC_AUX_12 = Telemetry_SetBit(15),
} Telemetry_Receiver_TagList;

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
    uint16_t min;
    uint16_t max;
    uint16_t *channel_ptr;
} Telemetry_ChannelSet_TypeDef;

typedef struct
{
    item_obj combo_list;
    uint8_t combo_cnt;
} Telemetry_RCFuncMap_TypeDef;

typedef struct
{
    bool init_state;
    bool arm_state;
    uint8_t control_mode;
    uint16_t module_enable;
    uint32_t aux_func_reg;

    Telemetry_RCFuncMap_TypeDef Gimbal[Telemetry_Gimbal_TagSum];
    Telemetry_RCFuncMap_TypeDef ARM_Toggle;
    Telemetry_RCFuncMap_TypeDef DisARM_Toggle;
    Telemetry_RCFuncMap_TypeDef ControlMode_Toggle;
    Telemetry_RCFuncMap_TypeDef Buzzer_Toggle;
    Telemetry_RCFuncMap_TypeDef Log_Toggle;
} Telemetry_RCInput_TypeDef;
#pragma pack()

void TaskTelemetry_Init(void);
void TaskTelemetry_Core(Task_Handle hdl);

#endif
