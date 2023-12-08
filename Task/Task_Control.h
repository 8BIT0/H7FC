#ifndef __TASK_CONTROL_H
#define __TASK_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "imu_data.h"
#include "cmsis_os.h"
#include "pid.h"
#include "../common/util.h"

#define TASKCONTROL_SET_BIT(x) UTIL_SET_BIT(x)
#define IMU_ERROR_UPDATE_MAX_COUNT 10
#define IMU_NONE_UPDATE_THRESHOLD 10
#define OVER_ANGULAR_ACCELERATE_COUNT 5

#define CLI_MESSAGE_OPEARATE_TIMEOUT 1

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

typedef struct
{
    bool init_state;
    bool control_abort;
    bool angular_protect;
    bool CLI_enable;

    uint8_t angular_warning_cnt;

    uint8_t ctl_model;

    uint8_t imu_update_error_cnt;
    uint8_t rc_update_error_cnt;

    uint32_t IMU_Rt;
    uint32_t RC_Rt;
    uint32_t ATT_Rt;

    bool auto_control;

    float acc_scale;
    float gyr_scale;

    float acc[Axis_Sum];
    float gyr[Axis_Sum];
    float imu_tmpr;

    float acc_lst[Axis_Sum];
    float gyr_lst[Axis_Sum];

    IMUAtt_TypeDef attitude; 

    uint32_t error_code;
    uint8_t imu_none_update_cnt;
    uint8_t over_angular_accelerate_cnt;

    bool att_pid_state;

    /* outer ring attitude control pid */
    PIDObj_TypeDef RollCtl_PIDObj;
    PIDObj_TypeDef PitchCtl_PIDObj;
    PIDObj_TypeDef YawCtl_PIDObj;

    /* inner ring angular speed control pid */
    PIDObj_TypeDef GyrXCtl_PIDObj;
    PIDObj_TypeDef GyrYCtl_PIDObj;
    PIDObj_TypeDef GyrZCtl_PIDObj;

    osMessageQId CLIMessage_ID;
} TaskControl_Monitor_TypeDef;

typedef enum
{
    TaskControl_Moto_CliDisable = 0,
    TaskControl_Moto_Set_Spin,
    TaskControl_Moto_Set_Dir,
    TaskControl_Servo_Set_Spin,
    TaskControl_Servo_Set_Dir,
} TaskControl_CLIDataType_List;

typedef struct
{
    uint32_t timestamp;
    TaskControl_CLIDataType_List cli_type;
    uint8_t index;
    uint16_t value_list[8];
} TaskControl_CLIData_TypeDef;

void TaskControl_Init(uint32_t period);
void TaskControl_Core(void const *arg);

#endif