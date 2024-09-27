#ifndef __SRV_DATAHUB_H
#define __SRV_DATAHUB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "DataPipe.h"
#include "pos_data.h"
#include "Srv_Actuator.h"
#include "Srv_IMUSample.h"
#include "Srv_Receiver.h"
#include "Srv_SensorMonitor.h"

#define SRVDATAHUB_TUNNING_HEARTBEAT_TIMEOUT 3000  /* unit: ms 3S timeout */
#define SRVDATAHUB_CONFIGRATOR_ATTACH_TIMEOUT 2000 /* unit: ms 2S timeout */

#define DRONE_ARM 1
#define DRONE_DISARM 0

typedef union
{
    struct
    {
        uint64_t raw_imu : 1;               // bit : 1
        uint64_t scaled_imu : 1;            // bit : 2
        uint64_t range_imu : 1;             // bit : 3
        
        uint64_t raw_mag : 1;               // bit : 4
        uint64_t scaled_mag : 1;            // bit : 5
        uint64_t range_mag : 1;             // bit : 6
        
        uint64_t scaled_baro : 1;           // bit : 7
        uint64_t range_baro : 1;            // bit : 8
        
        uint64_t raw_sonar : 1;             // bit : 9
        uint64_t scaled_sonar : 1;          // bit : 10
        uint64_t range_sonar : 1;           // bit : 11
        
        uint64_t raw_tof : 1;               // bit : 12
        uint64_t scaled_tof : 1;            // bit : 13
        uint64_t range_tof : 1;             // bit : 14
        
        uint64_t raw_flow : 1;              // bit : 15
        uint64_t scale_flow : 1;            // bit : 16
        uint64_t range_flow : 1;            // bit : 17
        
        uint64_t cnv_control_data : 1;      // bit : 18
        uint64_t rc_control_data : 1;       // bit : 19
        
        uint64_t actuator : 1;              // bit : 20
        uint64_t attitude : 1;              // bit : 21
        uint64_t relative_alt : 1;          // bit : 22

        uint64_t mag_init : 1;              // bit : 23
        uint64_t imu_init : 1;              // bit : 24
        uint64_t baro_init : 1;             // bit : 25
        
        uint64_t USB_VCP_attach : 1;        // bit : 26

        uint64_t cli : 1;                   // bit : 27
        uint64_t upgrade : 1;               // bit : 28
    } bit;

    uint64_t val;
} SrvDataHub_UpdateReg_TypeDef;

typedef struct
{
    bool imu_init_state;
    uint32_t imu_update_time;
    float acc_scale;
    float gyr_scale;

    uint8_t imu_num;
    uint8_t cur_use_imu;

    uint8_t pri_acc_range;
    uint16_t pri_gyr_range;

    uint8_t sec_acc_range;
    uint16_t sec_gyr_range;

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
    float baro_range;
    float baro_alt;
    float baro_pressure;
    float baro_tempra;
    float baro_alt_offset;
    uint8_t baro_error_code;

    uint32_t att_update_time;
    float att_roll;
    float att_pitch;
    float att_yaw;
    float att_q0;
    float att_q1;
    float att_q2;
    float att_q3;
    bool att_flip_over;
    uint8_t att_error_code;

    /* altitude relative to lift off place */
    uint32_t relative_alt_time;
    float relative_alt;

    ControlData_TypeDef RC_Control_Data;

    uint32_t cnvctl_data_time;
    bool arm;
    bool failsafe;
    uint8_t ctl_mode;
    uint8_t throttle_percent;
    float exp_pitch;
    float exp_roll;
    float exp_gyr_x;
    float exp_gyr_y;
    float exp_gyr_z;

    bool pos_enable;
    bool pos_init_state;
    uint32_t pos_update_time;
    double pos_x;
    double pos_y;
    double pos_z;

    double x_vel;
    double y_vel;
    double z_vel;

    bool flow_enable;
    bool flow_init_state;
    uint32_t flow_update_time;
    float relative_pos_x;
    float relative_pos_y;
    float relative_vel_x;
    float relative_vel_y;

    uint32_t actuator_update_time;
    uint8_t moto_num;
    uint8_t servo_num;
    uint16_t moto[8];
    uint8_t servo[8];

    /* reserved */
    uint32_t tunning_heartbeat_timestamp;
    bool in_tunning;
    uint32_t tunning_port_addr;

    bool upgrading;

    bool CLI_state;
    bool VCP_Attach;
} SrvDataHubObj_TypeDef;

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
    bool (*set_vcp_attach_state)(bool state);
    bool (*set_cli_state)(bool state);
    bool (*set_upgrade_state)(bool state);

    bool (*get_upgrade_state)(bool *state);
    bool (*get_cli_state)(bool *state);
    bool (*get_vcp_attach_state)(bool *state);
    bool (*get_imu_init_state)(bool *state);
    bool (*get_pri_imu_range)(uint8_t *acc_range, uint16_t *gyr_range);
    bool (*get_sec_imu_range)(uint8_t *acc_range, uint16_t *gye_range);
    bool (*get_baro_init_state)(bool *state);
    bool (*get_mag_init_state)(bool *state);
    bool (*get_relative_alt)(uint32_t *time_stamp, float *alt);
    bool (*get_raw_imu)(uint32_t *time_stamp, float *acc_scale, float *gyr_scale, float *acc_x, float *acc_y, float *acc_z, float *gyr_x, float *gyr_y, float *gyr_z, float *tmp, uint8_t *err);
    bool (*get_scaled_imu)(uint32_t *time_stamp, float *acc_scale, float *gyr_scale, float *acc_x, float *acc_y, float *acc_z, float *gyr_x, float *gyr_y, float *gyr_z, float *tmp, uint8_t *err);
    bool (*get_raw_mag)(uint32_t *time_stamp, float *scale, float *mag_x, float *mag_y, float *mag_z, uint8_t *err);
    bool (*get_scaled_mag)(uint32_t *time_stamp, float *scale, float *mag_x, float *mag_y, float *mag_z, uint8_t *err);
    bool (*get_attitude)(uint32_t *time_stamp, float *pitch, float *roll, float *yaw, float *q0, float *q1, float *q2, float *q3, bool *flip_over);
    bool (*get_rc_control_data)(ControlData_TypeDef *data);
    bool (*get_cnv_control_data)(uint32_t *time_stamp, bool *arm, bool *failsafe, uint8_t *mode, float *pitch, float *roll, float *gx, float *gy, float *gz);
    bool (*get_baro_altitude)(uint32_t *time_stamp, float *baro_pressure, float *baro_alt, float *baro_alt_offset, float *baro_temp, uint8_t *error);
    bool (*get_arm_state)(bool *arm);
    bool (*get_failsafe)(bool *failsafe);
    bool (*get_moto)(uint32_t *time_stamp, uint8_t *cnt, uint16_t *ch, uint8_t *dir);
} SrvDataHub_TypeDef;

extern SrvDataHub_TypeDef SrvDataHub;

#ifdef __cplusplus
}
#endif

#endif
