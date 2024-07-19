#ifndef __TASK_TELEMETRY_H
#define __TASK_TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "Srv_Receiver.h"
#include "linked_list.h"
#include "../common/util.h"
#include "semphr.h"
#include "HW_Def.h"
#include "Srv_ComProto.h"
#include "Srv_OsCommon.h"
#include "Bsp_USB.h"
#include "Bsp_Uart.h"
#include "Srv_DataHub.h"
#include "control_data.h"

#define Telemetry_SetBit(x) UTIL_SET_BIT(x)

#define TELEMETRY_SET_ARM DRONE_ARM
#define TELEMETRY_SET_DISARM DRONE_DISARM
#define TELEMETRY_DISABLE_ALL_MODULE 0
#define TELEMETRY_RCSIG_MAX_COMOBO_CNT 8

#define TELEMETRY_RC_CHANNEL_RANGE_MIN CHANNEL_RANGE_MIN
#define TELEMETRY_RC_CHANNEL_RANGE_MID CHANNEL_RANGE_MID
#define TELEMETRY_RC_CHANNEL_RANGE_MAX CHANNEL_RANGE_MAX

#define TELEMETRY_RC_GIMBAL_ZERO_ZONE_RANGE 20

#define TELEMETRY_RC_THROTTLE_PERCENT_ALERT 5

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
    uint16_t center_deadzone_scope;

    uint16_t min;
    uint16_t mid;
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

    Telemetry_RCFuncMap_TypeDef Gimbal[Gimbal_Sum];
    Telemetry_RCFuncMap_TypeDef ARM_Toggle;
    Telemetry_RCFuncMap_TypeDef ControlMode_Toggle;
    Telemetry_RCFuncMap_TypeDef Buzzer_Toggle;
    Telemetry_RCFuncMap_TypeDef Blackbox_Toggle;
    Telemetry_RCFuncMap_TypeDef FlipOver_Toggle;
    Telemetry_RCFuncMap_TypeDef Log_Toggle;
    Telemetry_RCFuncMap_TypeDef CLB_Toggle;
} Telemetry_RCInput_TypeDef;

typedef struct
{
    uint32_t Init_Rt;
    uint32_t Start_RunCore_Rt;
    uint32_t OnStart_SelfSDetect_Duration;

    int16_t receiver_value_max;
    int16_t receiver_value_mid;
    int16_t receiver_value_min;

    uint8_t throttle_ch;
    uint8_t pitch_ch;
    uint8_t roll_ch;
    uint8_t yaw_ch;

    uint8_t arm_toggle_ch;
    uint8_t buzzer_toggle_ch;
    uint8_t mode_switcher_ch;
    uint8_t blackbox_ch;
    uint8_t flip_over_ch;

    bool poweron_arm_check;

    uint32_t failsafe_trigger_cnt;
    bool recover_failsafe;
    bool lst_arm_state;
 
    Telemetry_RCInput_TypeDef RC_Setting;
} Telemetry_Monitor_TypeDef;

void TaskTelemetry_Init(uint32_t period);
void TaskTelemetry_Core(void const* arg);

#ifdef __cplusplus
}
#endif

#endif
