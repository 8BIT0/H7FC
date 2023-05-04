#ifndef __DEV_ICM20602_H
#define __DEV_ICM20602_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include "imu_data.h"

#define ICM20602_RESET_TIMEOUT 100

#define ICM20602_READ_MASK 0x80
#define ICM20602_DEV_ID 0x12 // SA0 attach with GND then Device ID is 0x68 else if SA0 Pull Up 0x69 (default)

#define ICM20602_RESET_CMD 0x80
#define ICM20602_RESET_SUCCESS 0x41

#define ICM20602_XG_OFFS_TC_H 0x04
#define ICM20602_XG_OFFS_TC_L 0x05
#define ICM20602_YG_OFFS_TC_H 0x07
#define ICM20602_YG_OFFS_TC_L 0x08
#define ICM20602_ZG_OFFS_TC_H 0x0A
#define ICM20602_ZG_OFFS_TC_L 0x0B
#define ICM20602_SELF_TEST_X_ACCEL 0x0D
#define ICM20602_SELF_TEST_Y_ACCEL 0x0E
#define ICM20602_SELF_TEST_Z_ACCEL 0x0F
#define ICM20602_XG_OFFS_USRH 0x13
#define ICM20602_XG_OFFS_USRL 0x14
#define ICM20602_YG_OFFS_USRH 0x15
#define ICM20602_YG_OFFS_USRL 0x16
#define ICM20602_ZG_OFFS_USRH 0x17
#define ICM20602_ZG_OFFS_USRL 0x18
#define ICM20602_SMPLRT_DIV 0x19
#define ICM20602_CONFIG 0x1A
#define ICM20602_GYRO_CONFIG 0x1B
#define ICM20602_ACCEL_CONFIG 0x1C
#define ICM20602_ACCEL_CONFIG_2 0x1D
#define ICM20602_LP_MODE_CFG 0x1E
#define ICM20602_ACCEL_WOM_X_THR 0x20
#define ICM20602_ACCEL_WOM_Y_THR 0x21
#define ICM20602_ACCEL_WOM_Z_THR 0x22
#define ICM20602_FIFO_EN 0x23
#define ICM20602_FSYNC_INT 0x36
#define ICM20602_INT_PIN_CFG 0x37
#define ICM20602_INT_ENABLE 0x38
#define ICM20602_FIFO_WM_INT_STATUS 0x39
#define ICM20602_INT_STATUS 0x3A
#define ICM20602_ACCEL_XOUT_H 0x3B
#define ICM20602_ACCEL_XOUT_L 0x3C
#define ICM20602_ACCEL_YOUT_H 0x3D
#define ICM20602_ACCEL_YOUT_L 0x3E
#define ICM20602_ACCEL_ZOUT_H 0x3F
#define ICM20602_ACCEL_ZOUT_L 0x40
#define ICM20602_TEMP_OUT_H 0x41
#define ICM20602_TEMP_OUT_L 0x42
#define ICM20602_GYRO_XOUT_H 0x43
#define ICM20602_GYRO_XOUT_L 0x44
#define ICM20602_GYRO_YOUT_H 0x45
#define ICM20602_GYRO_YOUT_L 0x46
#define ICM20602_GYRO_ZOUT_H 0x47
#define ICM20602_GYRO_ZOUT_L 0x48
#define ICM20602_SELF_TEST_X_GYRO 0x50
#define ICM20602_SELF_TEST_Y_GYRO 0x51
#define ICM20602_SELF_TEST_Z_GYRO 0x52
#define ICM20602_FIFO_WM_TH1 0x60
#define ICM20602_FIFO_WM_TH2 0x61
#define ICM20602_SIGNAL_PATH_RESET 0x68
#define ICM20602_ACCEL_INTEL_CTRL 0x69
#define ICM20602_USER_CTRL 0x6A
#define ICM20602_PWR_MGMT_1 0x6B
#define ICM20602_PWR_MGMT_2 0x6C
#define ICM20602_I2C_IF 0x70
#define ICM20602_FIFO_COUNTH 0x72
#define ICM20602_FIFO_COUNTL 0x73
#define ICM20602_FIFO_R_W 0x74
#define ICM20602_WHO_AM_I 0x75
#define ICM20602_XA_OFFSET_H 0x77
#define ICM20602_XA_OFFSET_L 0x78
#define ICM20602_YA_OFFSET_H 0x7A
#define ICM20602_YA_OFFSET_L 0x7B
#define ICM20602_ZA_OFFSET_H 0x7D
#define ICM20602_ZA_OFFSET_L 0x7E

#define ICM20602_ACC_16G_SCALE 2048.0
#define ICM20602_ACC_8G_SCALE 4096.0
#define ICM20602_ACC_4G_SCALE 8192.0
#define ICM20602_ACC_2G_SCALE 16384.0

#define ICM20602_GYR_2000DPS_SCALE 16.4
#define ICM20602_GYR_1000DPS_SCALE 32.8
#define ICM20602_GYR_500DPS_SCALE 65.5
#define ICM20602_GYR_250DPS_SCALE 131.0

typedef enum
{
    ICM20602_SampleRate_8K = 0,
    ICM20602_SampleRate_4K = 1,
    ICM20602_SampleRate_2K = 3,
    ICM20602_SampleRate_1K = 7,
} ICM20602_SampleRate_List;

typedef enum
{
    ICM20602_Gyr_250DPS = 0,
    ICM20602_Gyr_500DPS,
    ICM20602_Gyr_1000DPS,
    ICM20602_Gyr_2000DPS
} ICM20602_GyrTrip_List;

typedef enum
{
    ICM20602_Acc_2G = 0,
    ICM20602_Acc_4G,
    ICM20602_Acc_8G,
    ICM20602_Acc_16G,
} ICM20602_AccTrip_List;

typedef enum
{
    ICM20602_No_Error = 0,
    ICM20602_Obj_Error,
    ICM20602_DevID_Error,
    ICM20602_Reset_Error,
    ICM20602_OSC_Error,
    ICM20602_InertialEnable_Error,
    ICM20602_InterfaceSet_Error,
    ICM20602_AccSet_Error,
    ICM20602_GyrSet_Error,
    ICM20602_RateSet_Error,
} ICM20602_Error_List;

typedef struct
{
    cs_ctl_callback cs_ctl;
    bus_trans_callback bus_trans;
    delay_callback delay;
    get_time_stamp_callback get_timestamp;

    bool drdy;
    bool update;

    uint8_t AccTrip;
    uint8_t GyrTrip;

    uint8_t PHY_AccTrip_Val;
    uint16_t PHY_GyrTrip_Val;

    float acc_scale;
    float gyr_scale;

    ICM20602_Error_List error;
    ICM20602_SampleRate_List rate;

    IMUData_TypeDef OriData;
} DevICM20602Obj_TypeDef;

typedef struct
{
    bool (*detect)(bus_trans_callback trans, cs_ctl_callback cs_ctl);
    void (*config)(DevICM20602Obj_TypeDef *Obj, ICM20602_SampleRate_List rate, ICM20602_GyrTrip_List GyrTrip, ICM20602_AccTrip_List AccTrip);
    void (*pre_init)(DevICM20602Obj_TypeDef *Obj, cs_ctl_callback cs_ctl, bus_trans_callback bus_trans, delay_callback delay, get_time_stamp_callback get_time_stamp);
    bool (*init)(DevICM20602Obj_TypeDef *Obj);
    void (*set_ready)(DevICM20602Obj_TypeDef *Obj);
    bool (*get_ready)(DevICM20602Obj_TypeDef *Obj);
    bool (*reset)(DevICM20602Obj_TypeDef *Obj);
    bool (*sample)(DevICM20602Obj_TypeDef *Obj);
    IMUData_TypeDef (*get_data)(DevICM20602Obj_TypeDef *Obj);
    ICM20602_Error_List (*get_error)(DevICM20602Obj_TypeDef *Obj);
    IMUModuleScale_TypeDef (*get_scale)(const DevICM20602Obj_TypeDef *sensor_obj);
    float (*get_gyr_angular_speed_diff)(const DevICM20602Obj_TypeDef *sensor_obj);
} DevICM20602_TypeDef;

extern DevICM20602_TypeDef DevICM20602;

#endif
