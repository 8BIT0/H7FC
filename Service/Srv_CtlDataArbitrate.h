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
} Srv_CtlSigMode_List;

typedef enum
{
    Control_Mode_Attitude = 0,
    Control_Mode_AngularSpeed,
    Control_Mode_AngluarSpeed_AngleLock,
    Control_Mode_Manul,
} Srv_CtlMode_List;

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
    uint16_t rc_channel[MAX_RECEIVER_CHANNEL_NUM];

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
    uint8_t ctl_mode;
    uint8_t RC_TakingOver_percent;

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

    float dead_zone_max;
    float dead_zone_min;

    float max_bias_to_lst;
    float min_bias_to_lst;
} Srv_CtlRange_TypeDef;

typedef struct
{
    Srv_CtlNegociateData_TypeDef Data;
    
    Srv_CtlRange_TypeDef att_ctl_range[2];
    Srv_CtlRange_TypeDef angularspeed_ctl_range[3];

} Srv_CtlArbitrateMonitor_TypeDef;

typedef struct
{
    bool (*init)(Srv_CtlRange_TypeDef att_range[2], Srv_CtlRange_TypeDef angularspeed_range[3]);
    bool (*negociate_update)(void);
    Srv_CtlExpectionData_TypeDef (*get_data)(void);
} Srv_CtlDataArbitrate_TypeDef;

extern Srv_CtlDataArbitrate_TypeDef Srv_CtlDataArbitrate;

#endif
