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

typedef union
{
    uint16_t val;
    struct
    {
        uint16_t imu     : 1;   /* bit1: imu data */
        uint16_t baro    : 1;   /* bit2: baro data */
        uint16_t att     : 1;   /* bit3: attitude data */
        uint16_t alt     : 1;   /* bit4: altitude data */
        uint16_t exp_ctl : 1;   /* bit5: expection control data, convert from rc receiver */
        uint16_t act     : 1;   /* bit6: actuator data */
        uint16_t res     : 10;
    } bit;
} BlackBox_Reg_TypeDef;

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

typedef struct
{
    uint32_t time;
    uint8_t cyc;

    uint16_t ch[8];
} BlackBox_ActuatorData_TypeDef;
#pragma pack()

void TaskBlackBox_Init(void);
void TaskBlackBox_Core(void const *arg);

#ifdef __cplusplus
}
#endif

#endif

