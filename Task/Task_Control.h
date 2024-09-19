#ifndef __TASK_CONTROL_H
#define __TASK_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "imu_data.h"
#include "Srv_OsCommon.h"
#include "pid.h"
#include "../System/storage/Storage.h"
#include "controller.h"
#include "../common/util.h"

#define TASKCONTROL_SET_BIT(x) UTIL_SET_BIT(x)
#define IMU_ERROR_UPDATE_MAX_COUNT 10
#define IMU_NONE_UPDATE_THRESHOLD 10
#define OVER_ANGULAR_ACCELERATE_COUNT 5

#define CLI_MESSAGE_OPEARATE_TIMEOUT 1

typedef enum
{
    Moto_Lock = 0,
    Moto_Unlock,
    Moto_Unlock_Err,
} TaskControl_MotoUnlock_State;

typedef union
{
    struct
    {
        uint32_t rc : 1;
        uint32_t imu : 1;

        uint32_t reserve : 30;
    }Sec;

    uint32_t val;
}TaskControl_ErrReg_TypeDef;

typedef enum
{
    TaskControl_None_Error = 0,

    /* IMU Section */
    TaskControl_IMU_NoneUpdate = TASKCONTROL_SET_BIT(0),
    TaskControl_IMU_DataBlunt = TASKCONTROL_SET_BIT(1),
    TaskControl_IMU_OverAngularAcc = TASKCONTROL_SET_BIT(2),
    
    /* RC Section */
    TaskControl_RC_FailSafe = TASKCONTROL_SET_BIT(3),
    TaskControl_RC_LowSig = TASKCONTROL_SET_BIT(4),
    TaskControl_RC_ErrInput = TASKCONTROL_SET_BIT(5),
}TaskControl_ErrorCode_List;

#pragma pack(1)
typedef struct
{
    ControlMode_List mode;

    float att_rate;
    float pitch_range;
    float roll_range;

    float gx_rate;
    float gx_range;

    float gy_rate;
    float gy_range;

    float gz_rate;
    float gz_range;
} TaskControl_CtlPara_TypeDef;
#pragma pack()

typedef struct
{
    bool init_state;
    bool control_abort;
    bool angular_protect_enable;
    bool angular_protect;
    bool throttle_protect_enable;
    bool throttle_protect;
    bool CLI_enable;

    Storage_ItemSearchOut_TypeDef pid_store_info;
    TaskControl_CtlPara_TypeDef ctl_range;

    Storage_ItemSearchOut_TypeDef actuator_store_info;
    SrvActuator_Setting_TypeDef actuator_param;

    uint8_t angular_warning_cnt;
    uint8_t imu_update_error_cnt;

    uint32_t IMU_Rt;
    uint32_t ATT_Rt;

    float acc[Axis_Sum];
    float gyr[Axis_Sum];

    float acc_lst[Axis_Sum];
    float gyr_lst[Axis_Sum];

    uint8_t throttle_percent;
    float exp_gyr_x;
    float exp_gyr_y;
    float exp_gyr_z;
    float exp_pitch;
    float exp_roll;

    IMUAtt_TypeDef attitude;
    bool flip_over;
    bool dynamic_disarm_enable;
    TaskControl_MotoUnlock_State moto_unlock;

    uint32_t error_code;
    uint8_t imu_none_update_cnt;
    uint8_t over_angular_accelerate_cnt;

    osMessageQId CLIMessage_ID;
} TaskControl_Monitor_TypeDef;

typedef enum
{
    Actuator_Spin_ClockWise = 0,
    Actuator_Spin_AntiClockWise,
} TaskControl_ActuatorSpinDir_List;

typedef enum
{
    TaskControl_Moto_CliDisable = 0,
    TaskControl_Moto_Set_Spin,
    TaskControl_Moto_Set_SpinDir,
    TaskControl_Servo_Set_Spin,
    TaskControl_Servo_Set_SpinDir,
} TaskControl_CLIDataType_List;

typedef struct
{
    uint32_t timestamp;
    TaskControl_CLIDataType_List cli_type;
    uint8_t index;
    uint16_t value;
} TaskControl_CLIData_TypeDef;

void TaskControl_Init(uint32_t period);
void TaskControl_Core(void const *arg);

#ifdef __cplusplus
}
#endif

#endif
