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
    BlackBox_Log_None = 0,
    BlackBox_Imu_Filted,        /* log raw gyr acc and filted gyr acc data                                      rate for 1Khz */
    BlackBox_Log_Alt_Att,       /* log altitude and attitude data                                               rate for 100Hz */
    BlackBox_AngularPID_Tune,   /* log filted gyr and expection gyr data                                        rate for 500Hz */
    BlackBox_AttitudePID_Tune,  /* log pitch roll and z_angular and expection pitch roll and z_angular data     rate for 100Hz */
} BlackBox_LogType_List;

typedef struct
{
    uint32_t time;
    uint8_t cyc;

    float press;
    uint8_t throttle_percent;
} BlackBox_BaroData_TypeDef;

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
    int16_t org_acc[Axis_Sum];
    int16_t org_gyr[Axis_Sum];
    int16_t flt_acc[Axis_Sum];
    int16_t flt_gyr[Axis_Sum];
    uint8_t throttle_percent;
} BlackBox_IMUData_TypeDef;

typedef struct
{
    uint32_t time;
    uint8_t throttle_percent;
    float gyr[Axis_Sum];
    float exp_gyr[Axis_Sum];
} BlackBox_AngCtlData_TypeDef;

typedef struct
{
    uint32_t time;
    uint8_t throttle_percent;
    float pitch;
    float roll;
    float gyr_z;
    float exp_pitch;
    float exp_roll;
    float exp_gyr_z;
} BlackBox_AttCtlData_TypeDef;

typedef struct
{
    uint32_t time;

    float baro;
    float acc_scale;
    float gyr_scale;
    int16_t acc[Axis_Sum];
    int16_t gyr[Axis_Sum];

    float pitch;
    float roll;
    float yaw;
    float alt;
} BlackBox_AttAltData_TypeDef;
#pragma pack()

void TaskBlackBox_Init(void);
void TaskBlackBox_Core(void const *arg);
void TaskBlackBox_LogControl(void);
bool TaskBlackBox_Set_LogInfo(BlackBox_MediumType_List medium, BlackBox_LogType_List type, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif

