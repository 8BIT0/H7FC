#ifndef SRV_CTLDATAARBITRATE_H
#define SRV_CTLDATAARBITRATE_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "Srv_DataHub.h"

typedef enum
{
    Control_Sig_RC = 0,
    Control_Sig_OnPlaneComputer,
} Srv_CtlSigSrcMode_List;

typedef enum
{
    Control_Channel_Sig = 0,
    Control_Attitude_Sig,
    Control_AngularSpeed_Sig,
} Srv_CtlSigInputType_List;

typedef enum
{
    Control_Mode_Attitude = 0,
    Control_Mode_AngularSpeed,
    Control_Mode_AngluarSpeed_AngleLock,
} Srv_CtlMode_List;

typedef enum
{
    ArbitrateState_None = 0,
    ArbitrateState_InProcess,
    ArbitrateState_Done,
    ArbitrateState_Denied,
} Srv_CtlArbitrateState_List;

typedef struct
{
    bool tunning;
    bool attach_configrator;

    bool arm_state;
    bool buzzer_state;
    bool calib_state;
    bool fail_safe;

    uint8_t ctl_mode;
    uint8_t RC_TakingOver_percent;

    uint8_t rc_channel_num;
    uint16_t rc_channel[Channel_Max];

    uint8_t idle_throttle_percent;
    uint8_t throttle_percent;

    float exp_alt;
    float exp_attitude[Axis_Sum];
    float exp_angular_speed[Axis_Sum];
} Srv_CtlNegociateData_TypeDef;

typedef struct
{
    bool tunning;
    bool attach_configrator;

    bool arm_state;
    bool buzzer_state;
    bool calib_state;
    bool fail_safe;
    Srv_CtlMode_List ctl_mode;
    uint8_t TakingOver_stage; /* signal arbitrate and sync process stage */

    uint8_t idle_throttle_percent;
    uint8_t throttle_percent;

    float exp_attitude[Axis_Sum];
    float exp_angularspeed[Axis_Sum];
    float exp_alt;
} Srv_CtlExpectionData_TypeDef;

typedef struct
{
    float max;
    float idle;
    float min;

    bool enable_dead_zone;
    float dead_zone_max;
    float dead_zone_min;
} Srv_CtlRange_TypeDef;

typedef struct
{
    Srv_CtlNegociateData_TypeDef Data;
    
    Srv_CtlRange_TypeDef att_ctl_range[Att_Ctl_Sum];
    Srv_CtlRange_TypeDef angularspeed_ctl_range[Axis_Sum];

    Srv_CtlSigSrcMode_List cur_sig_source;
    Srv_CtlSigInputType_List cur_sig_type;
    Srv_CtlMode_List cur_ctl_mode;

    Srv_CtlArbitrateState_List arbitrate_state;
    Srv_CtlSigSrcMode_List sig_privilege_req_source;

    ControlData_TypeDef RC_CtlData;
    ControlData_TypeDef OPC_CtlData;
    ControlData_TypeDef InUse_CtlData;
} Srv_CtlArbitrateMonitor_TypeDef;

typedef struct
{
    bool (*init)(Srv_CtlRange_TypeDef att_range[Att_Ctl_Sum], Srv_CtlRange_TypeDef angularspeed_range[Axis_Sum]);
    bool (*negociate_update)(void);
    Srv_CtlExpectionData_TypeDef (*get_data)(void);
} Srv_CtlDataArbitrate_TypeDef;

extern Srv_CtlDataArbitrate_TypeDef Srv_CtlDataArbitrate;

#endif
