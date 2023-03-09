#ifndef __SRV_DATAHUB_H
#define __SRV_DATAHUB_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "DataPipe.h"
#include "Srv_Actuator.h"
#include "Srv_IMUSample.h"
#include "Srv_Receiver.h"

#pragma pack(1)
typedef union
{
    struct
    {
        uint8_t imu_updating : 1;
        uint8_t mag_updating : 1;
        uint8_t baro_updating : 1;
        uint8_t sonar_updating : 1;
        uint8_t tof_updating : 1;
        uint8_t rc_updating : 1;
    } bit;

    uint8_t val;
} SrvDataHub_UpdateReg_TypeDef;

typedef struct
{
    uint32_t imu_update_time;
    float acc_scale;
    float gyr_scale;

    float flt_gyr_x;
    float flt_gyr_y;
    float flt_gyr_z;

    float org_gyr_x;
    float org_gyr_y;
    float org_gyr_z;

    float flt_acc_x;
    float flt_acc_y;
    float flt_acc_z;

    float org_acc_x;
    float org_acc_y;
    float org_acc_z;

    float imu_temp;

    uint32_t mag_update_time;
    float mag_scale;

    float flt_mag_x;
    float flt_mag_y;
    float flt_mag_z;

    float org_mag_x;
    float org_mag_y;
    float org_mag_z;

    float mag_temp;

    uint32_t baro_update_time;
    float baro_scale;
    float baro;

    uint32_t sonar_update_time;
    float sonar_scale;
    float sonar;

    uint32_t tof_update_time;
    float tof_scale;
    float tof;

    uint32_t att_update_time;
    float att_roll;
    float att_pitch;
    float att_yaw;

    uint32_t rc_update_time;
    uint16_t ch[16];

    uint32_t gnss_update_time;
    double lon;
    double lat;
    double alt;

    double vel_n;
    double vel_e;
    double vel_d;

    double forward_vel;
    double lateral_vel;
    double vertical_vel;

    uint16_t utc_year;
    uint16_t utc_month;
    uint16_t utc_day;
    uint16_t utc_hour;
    uint16_t utc_min;
    uint16_t utc_s;
    uint16_t utc_ms;

    uint32_t actucator_update_time;
    uint8_t moto_num;
    uint8_t servo_num;
    uint8_t moto_dir[8];
    uint8_t servo_dir[8];
    uint16_t moto[8];
    uint8_t servo[8];
} SrvDataHub_TypeDef;
#pragma pack()

#endif
