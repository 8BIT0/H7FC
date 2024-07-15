#ifndef __TASK_BLACKBOX_H
#define __TASK_BLACKBOX_H

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
    BlackBox_Medium_None = 0,
    BlackBox_Medium_Card,
    BlackBox_Medium_Chip,
    BlackBox_Medium_Com,
} BlackBox_MediumType_List;

typedef enum
{
    BlackBox_Imu_Filter = 0,    /* log raw gyr acc and filted gyr acc data                                      rate for 1Khz */
    BlackBox_AngularPID_Tune,   /* log filted gyr and expection gyr data                                        rate for 500Hz */
    BlackBox_AttitudePID_Tune,  /* log pitch roll and z_angular and expection pitch roll and z_angular data     rate for 100Hz */
    BlackBox_AltitudePID_Tune,  /* log raw baro altitude and expection altitude                                 rate for 100Hz */
} BlackBox_LogType_List;

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
    // int16_t org_acc[Axis_Sum];
    // int16_t org_gyr[Axis_Sum];
    int16_t flt_acc[Axis_Sum];
    int16_t flt_gyr[Axis_Sum];
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
#pragma pack()

void TaskBlackBox_Init(void);
void TaskBlackBox_Core(void const *arg);
void TaskBlackBox_LogControl(void);

#ifdef __cplusplus
}
#endif

#endif

