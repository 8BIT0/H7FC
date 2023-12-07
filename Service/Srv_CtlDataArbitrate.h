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
    uint8_t RC_TakingOver_percent;

    uint8_t rc_channel_num;
    uint16_t rc_channel[MAX_RECEIVER_CHANNEL_NUM];

    float Roll_LockAngle;
    float Pitch_LockAngle;

    uint8_t idle_throttle_percent;
    uint8_t throttle_percent;

    float exp_alt;
    float exp_attitude[Axis_Sum];
    float exp_angular_speed[Axis_Sum];
} Srv_CtlNegociateData_TypeDef;

typedef struct
{
    uint8_t throttle_percent;

    float exp_attitude[Axis_Sum];
    float exp_angularspeed[Axis_Sum];
    float exp_alt;
} Srv_CtlExpectionData_TypeDef;

typedef struct
{
    Srv_CtlNegociateData_TypeDef Data;
} Srv_CtlArbitrateMonitor_TypeDef;

typedef struct
{
    bool (*init)(void);
    bool (*negociate_update)(void);
    Srv_CtlExpectionData_TypeDef (*get_data)(void);
} Srv_CtlDataArbitrate_TypeDef;

extern Srv_CtlDataArbitrate_TypeDef Srv_CtlDataArbitrate;

#endif
