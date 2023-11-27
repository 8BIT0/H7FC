#ifndef __SRV_DATAHUB_H
#define __SRV_DATAHUB_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "DataPipe.h"
#include "Srv_Actuator.h"
#include "Srv_IMUSample.h"
#include "Srv_Receiver.h"
#include "Srv_SensorMonitor.h"

#define SRVDATAHUB_TUNNING_HEARTBEAT_TIMEOUT 3000  /* unit: ms 3S timeout */
#define SRVDATAHUB_CONFIGRATOR_ATTACH_TIMEOUT 2000 /* unit: ms 2S timeout */

#pragma pack(1)
typedef union
{
    struct
    {
        uint32_t raw_imu : 1;
        uint32_t scaled_imu : 1;

        uint32_t raw_mag : 1;
        uint32_t scaled_mag : 1;

        uint32_t raw_baro : 1;
        uint32_t scaled_baro : 1;

        uint32_t raw_sonar : 1;
        uint32_t scaled_sonar : 1;

        uint32_t raw_tof : 1;
        uint32_t scaled_tof : 1;

        uint32_t rc : 1;

        uint32_t actuator : 1;
        uint32_t attitude : 1;

        uint32_t mag_init : 1;
        uint32_t imu_init : 1;

        uint32_t tunning : 1;
        uint32_t configrator_attach : 1;

        uint32_t cli : 1;
    } bit;

    uint32_t val;
} SrvDataHub_UpdateReg_TypeDef;

typedef struct
{
    bool imu_init_state;
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
    uint8_t imu_error_code;

    bool mag_enabled;
    bool mag_init_state;
    uint32_t mag_update_time;
    float mag_scale;

    float flt_mag_x;
    float flt_mag_y;
    float flt_mag_z;

    float org_mag_x;
    float org_mag_y;
    float org_mag_z;

    float mag_temp;
    uint8_t mag_error_code;

    bool baro_enabled;
    bool baro_init_state;
    uint32_t baro_update_time;
    float baro_scale;
    float baro;
    uint8_t baro_error_code;

    bool tof_enabled;
    bool tof_init_state;
    uint32_t tof_update_time;
    float tof_scale;
    float tof;
    uint8_t tof_error_code;

    uint32_t att_update_time;
    float att_roll;
    float att_pitch;
    float att_yaw;
    float att_q0;
    float att_q1;
    float att_q2;
    float att_q3;
    uint8_t att_error_code;

    uint32_t rc_update_time;
    bool arm;
    bool failsafe;
    bool osd_tune;
    bool buzz;
    bool module;
    uint8_t mode;
    uint8_t channel_sum;
    uint16_t ch[32];
    uint16_t gimbal[4];
    uint16_t link_quality;
    uint16_t rssi;
    
    bool gnss_enable;
    bool gnss_init_state;
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

    uint8_t gnss_error_code;

    uint32_t actuator_update_time;
    uint8_t moto_num;
    uint8_t servo_num;
    uint8_t moto_dir[8];
    uint8_t servo_dir[8];
    uint16_t moto[8];
    uint8_t servo[8];

    /* when receiver configrator`s heartbeat */
    uint32_t configrator_time_stamp;
    bool attach_configrator;

    uint32_t tunning_heartbeat_timestamp;
    bool in_tunning;
    uint32_t tunning_port_addr;

    bool CLI_state;
} SrvDataHubObj_TypeDef;
#pragma pack()

typedef struct
{
    bool init_state;
    SrvDataHub_UpdateReg_TypeDef inuse_reg;
    SrvDataHub_UpdateReg_TypeDef update_reg;
    SrvDataHubObj_TypeDef data;
} SrvDataHub_Monitor_TypeDef;

typedef struct
{
    void (*init)(void);
    bool (*set_tunning_state)(uint32_t time_stamp, bool state, uint32_t port_addr);    /* set tunning status in can/uart/usb irq */
    bool (*set_configrator_state)(uint32_t time_stamp, bool state);
    bool (*set_cli_state)(bool state);

    bool (*get_cli_state)(bool *state);
    bool (*get_tunning_state)(uint32_t *time_stamp, bool *state, uint32_t *port_addr);
    bool (*get_configrator_attach_state)(uint32_t *time_stamp, bool *state);
    bool (*get_imu_init_state)(bool *state);
    bool (*get_baro_init_state)(bool *state);
    bool (*get_mag_init_state)(bool *state);
    bool (*get_tof_init_state)(bool *state);
    bool (*get_raw_imu)(uint32_t *time_stamp, float *acc_scale, float *gyr_scale, float *acc_x, float *acc_y, float *acc_z, float *gyr_x, float *gyr_y, float *gyr_z, float *tmp, uint8_t *err);
    bool (*get_scaled_imu)(uint32_t *time_stamp, float *acc_scale, float *gyr_scale, float *acc_x, float *acc_y, float *acc_z, float *gyr_x, float *gyr_y, float *gyr_z, float *tmp, uint8_t *err);
    bool (*get_raw_mag)(uint32_t *time_stamp, float *scale, float *mag_x, float *mag_y, float *mag_z, uint8_t *err);
    bool (*get_scaled_mag)(uint32_t *time_stamp, float *scale, float *mag_x, float *mag_y, float *mag_z, uint8_t *err);
    bool (*get_attitude)(uint32_t *time_stamp, float *pitch, float *roll, float *yaw, float *q0, float *q1, float *q2, float *q3);
    bool (*get_rc)(uint32_t *time_stamp, uint16_t *ch, uint8_t *ch_cnt);
    bool (*get_gimbal_percent)(uint16_t *gimbal);
    bool (*get_arm_state)(bool *arm);
    bool (*get_failsafe)(bool *failsafe);
    bool (*get_control_mode)(uint8_t *mode);
    bool (*get_moto)(uint32_t *time_stamp, uint8_t *cnt, uint16_t *ch, uint8_t *dir);
    bool (*get_servo)(uint32_t *time_stamp, uint8_t *cnt, uint16_t *ch, uint8_t *dir);
} SrvDataHub_TypeDef;

extern SrvDataHub_TypeDef SrvDataHub;

#endif
