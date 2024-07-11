#ifndef __TASK_EXTBLACKBOX_H
#define __TASK_EXTBLACKBOX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "imu_data.h"

#define BLACKBOX_LOG_HEADER 0xE11E
#define BLACKBOX_LOG_ENDER  0xF22F
#define BLACKBOX_HEADER_SIZE sizeof(BlackBox_DataHeader_TypeDef)
#define BLACKBOX_ENDER_SIZE sizeof(BlackBox_DataEnder_TypeDef)

typedef enum
{
    BlackBox_IMU = 0,
    BlackBox_Baro,
    BlackBox_CtlData,
    BlackBox_Attitude,
    BlackBox_Actuator,
} BlackBox_DataType_List;

#pragma pack(1)
typedef struct
{
    uint16_t header;
    uint8_t type;
    uint8_t size;
} BlackBox_DataHeader_TypeDef;

typedef struct
{
    uint8_t check_sum;
    uint16_t ender;
} BlackBox_DataEnder_TypeDef;

typedef struct
{
    uint32_t time;
    uint8_t cyc;
    float acc_scale;
    float gyr_scale;
    uint16_t org_acc[Axis_Sum];
    uint16_t org_gyr[Axis_Sum];
    uint16_t flt_acc[Axis_Sum];
    uint16_t flt_gyr[Axis_Sum];
} BlackBox_IMUData_TypeDef;

typedef struct
{
    uint32_t time;
    uint8_t cyc;

    float press;
} BlackBox_BaroData_TypeDef;

typedef struct
{
    uint32_t time;
    uint8_t cyc;

    bool arm_state;
    uint8_t mode;

    float exp_pitch;
    float exp_roll;

    float exp_gyrx;
    float exp_gyry;
    float exp_gyrz;
} BlackBox_CtlData_TypeDef;

typedef struct
{
    uint32_t time;
    uint8_t cyc;

    float pitch;
    float roll;
    float yaw;
} BlackBox_AttitudeData_TypeDef;

typedef struct
{
    uint32_t time;
    uint8_t cyc;

    uint16_t ch[8];
} BlackBox_ActuatorData_TypeDef;
#pragma pack()

void TaskExtBlackBox_Init(void);
void TaskExtBlackBox_Core(void const *arg);

#ifdef __cplusplus
}
#endif

#endif

