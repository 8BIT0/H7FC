#ifndef __TASK_TELEMETRY_H
#define __TASK_TELEMETRY_H

#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "../System/scheduler/scheduler.h"
#include "Srv_Receiver.h"
#include "linked_list.h"

#define Telemetry_SetBit(x) (1 << x)

#define TELEMETRY_SET_ARM 1
#define TELEMETRY_SET_DISARM 0
#define TELEMETRY_DISABLE_ALL_MODULE 0
#define TELEMETRY_RCSIG_MAX_COMOBO_CNT 8

#define TELEMETRY_OSD_TUNE_HOLD_TIME 5000 /* unit ms */

#define TELEMETRY_RC_CHANNEL_RANGE_MIN CHANNEL_RANGE_MIN
#define TELEMETRY_RC_CHANNEL_RANGE_MID CHANNEL_RANGE_MID
#define TELEMETRY_RC_CHANNEL_RANGE_MAX CHANNEL_RANGE_MAX

#define TELEMETRY_RC_GIMBAL_ZERO_ZONE_RANGE 20
#define TELEMETRY_OSDTUNE_POSHOLD 100 /* unit ms */ 

#pragma pack(1)
typedef enum
{
    Telemetry_RC_Throttle = 0,
    Telemetry_RC_Pitch,
    Telemetry_RC_Roll,
    Telemetry_RC_Yaw,
    Telemetry_Gimbal_TagSum,
    Telemetry_RC_AUX_1 = Telemetry_Gimbal_TagSum,
    Telemetry_RC_AUX_2,
    Telemetry_RC_AUX_3,
    Telemetry_RC_AUX_4,
    Telemetry_RC_AUX_5,
    Telemetry_RC_AUX_6,
    Telemetry_RC_AUX_7,
    Telemetry_RC_AUX_8,
    Telemetry_RC_AUX_9,
    Telemetry_RC_AUX_10,
    Telemetry_RC_AUX_11,
    Telemetry_RC_AUX_12,
} Telemetry_Receiver_TagList;

typedef enum
{
    Telemetry_Control_Mode_ACRO = 0,
    Telemetry_Control_Mode_STAB,
    Telemetry_Control_Mode_AUTO,
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
    uint8_t cnt;
    bool state;
    uint16_t pos; 
} Telemetry_ToggleData_TypeDef;

typedef struct
{
    bool enable_deadzone;
    uint16_t center_deadzone_min;
    uint16_t center_deadzone_max;

    uint16_t min;
    uint16_t max;
    uint16_t *channel_ptr;
    uint16_t reg; 
} Telemetry_ChannelSet_TypeDef;

typedef struct
{
    item_obj combo_list;
    uint8_t combo_cnt;
} Telemetry_RCFuncMap_TypeDef;

typedef SrvRecever_RCSig_TypeDef Telemetry_RCSig_TypeDef;

typedef struct
{
    bool init_state;
    uint32_t update_rt;
    uint32_t lst_update_rt;
    Telemetry_RCSig_TypeDef sig;
    uint32_t aux_func_reg;
    uint16_t rssi;
    uint16_t link_quality;

    Telemetry_RCFuncMap_TypeDef Gimbal[Telemetry_Gimbal_TagSum];
    Telemetry_RCFuncMap_TypeDef ARM_Toggle;
    Telemetry_RCFuncMap_TypeDef ControlMode_Toggle;
    Telemetry_RCFuncMap_TypeDef Buzzer_Toggle;
    Telemetry_RCFuncMap_TypeDef Log_Toggle;
    Telemetry_RCFuncMap_TypeDef OSD_Toggle;
} Telemetry_RCInput_TypeDef;
#pragma pack()

void TaskTelemetry_Init(void);
void TaskTelemetry_Core(Task_Handle hdl);

#endif
