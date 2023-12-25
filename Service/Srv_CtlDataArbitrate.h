#ifndef SRV_CTLDATAARBITRATE_H
#define SRV_CTLDATAARBITRATE_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "Srv_DataHub.h"

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
    float exp_attitude[Att_Ctl_Sum];
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
    Control_Mode_List mode;
    uint8_t TakingOver_stage; /* signal arbitrate and sync process stage */

    uint8_t idle_throttle_percent;
    uint8_t throttle_percent;

    float exp_attitude[Att_Ctl_Sum];
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

    ControlSig_ArbitrateState_List arbitrate_state;

    ControlData_TypeDef RC_CtlData;
    ControlData_TypeDef OPC_CtlData;
    ControlData_TypeDef InUse_CtlData;

    uint32_t RC_TakingOver_ReqHold;
} Srv_CtlArbitrateMonitor_TypeDef;

typedef struct
{
    bool (*init)(Srv_CtlRange_TypeDef att_range[Att_Ctl_Sum], Srv_CtlRange_TypeDef angularspeed_range[Axis_Sum]);
    void (*negociate_update)(ControlData_TypeDef *inuse_ctldata);
    Srv_CtlExpectionData_TypeDef (*get_data)(void);
} Srv_CtlDataArbitrate_TypeDef;

extern Srv_CtlDataArbitrate_TypeDef Srv_CtlDataArbitrate;

#endif
