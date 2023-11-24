#ifndef __TASK_TELEMETRY_H
#define __TASK_TELEMETRY_H

#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "cmsis_os.h"
#include "Srv_Receiver.h"
#include "linked_list.h"
#include "../common/util.h"
#include "semphr.h"
#include "Bsp_USB.h"
#include "Bsp_Uart.h"

#define Telemetry_SetBit(x) UTIL_SET_BIT(x)

#define RECEIVER_PORT UART4
#define RECEIVER_CRSF_RX_DMA Bsp_DMA_None               // Bsp_DMA_1
#define RECEIVER_CRSF_RX_DMA_STREAM Bsp_DMA_Stream_None // Bsp_DMA_Stream_4
#define RECEIVER_CRSF_TX_DMA Bsp_DMA_None               // Bsp_DMA_1
#define RECEIVER_CRSF_TX_DMA_STREAM Bsp_DMA_Stream_None // Bsp_DMA_Stream_5
#define RECEIVER_SBUS_RX_DMA Bsp_DMA_1
#define RECEIVER_SBUS_RX_DMA_STREAM Bsp_DMA_Stream_4
#define RECEIVER_SBUS_TX_DMA Bsp_DMA_1
#define RECEIVER_SBUS_TX_DMA_STREAM Bsp_DMA_Stream_5

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

#define TELEMETRY_RC_THROTTLE_PERCENT_ALERT 5

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

typedef struct
{
    uint32_t Init_Rt;
    uint32_t Start_RunCore_Rt;
    uint32_t OnStart_SelfSDetect_Duration;

    /* when power on and task detect arm set in disarm state in the first place */
    /* throw arm_warning */
    bool arm_alert;

    bool force_arm;

    uint32_t failsafe_trigger_cnt;
    bool recover_failsafe;
    bool lst_arm_state;
} Telemetry_Monitor_TypeDef;
#pragma pack()

typedef enum
{
    Telemetry_Bypass_None = 0,
    Telemetry_Bypass_RxOnly,
    Telemetry_Bypass_TxOnly,
    Telemetry_Bypass_BothDir,
} Telemetry_Bypass_TypeList;

typedef struct
{
    bool enable;
    osSemaphoreId_t p_tx_semphr;
    Telemetry_Bypass_TypeList bypass_mode;
    uint8_t *bypass_src;
}TelemetryPort_ByPass_TypeDef;

typedef struct
{
    bool init_state;

    BspUSB_VCP_TxStatistic_TypeDef tx_statistic;
    TelemetryPort_ByPass_TypeDef ByPass_Mode;
} Telemetry_VCPPortMonitor_TypeDef;

typedef struct
{
    bool init_state;
    TelemetryPort_ByPass_TypeDef ByPass_Mode;
} Telemetry_UartPortMonitor_TypeDef;

typedef struct
{
    bool init_state;
} Telemetry_CanPortMonitor_TypeDef;

typedef struct
{
    bool init;
    uint8_t uart_port_num;
    uint8_t can_port_num;

    void *Cur_Tuning_Port;

    Telemetry_VCPPortMonitor_TypeDef VCP_Port;
    Telemetry_UartPortMonitor_TypeDef *Uart_Port;
    Telemetry_CanPortMonitor_TypeDef *Can_Port;
} Telemetry_PortMonitor_TypeDef;

void TaskTelemetry_Init(uint32_t period);
void TaskTelemetry_Core(void const* arg);

#endif
