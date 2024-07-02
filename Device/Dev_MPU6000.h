#ifndef __DEV_MPU6000_H
#define __DEV_MPU6000_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include "imu_data.h"

#define MPU6000_WRITE_MASK 0x80

#define MPU6000_CONFIG 0x1A
#define MPU6000_PRODUCT_ID 0x0C
#define MPU6000_SMPLRT_DIV 0x19
#define MPU6000_GYRO_CONFIG 0x1B
#define MPU6000_ACCEL_CONFIG 0x1C
#define MPU6000_FIFO_EN 0x23
#define MPU6000_INT_PIN_CFG 0x37
#define MPU6000_INT_ENABLE 0x38
#define MPU6000_INT_STATUS 0x3A
#define MPU6000_ACCEL_XOUT_H 0x3B
#define MPU6000_ACCEL_XOUT_L 0x3C
#define MPU6000_ACCEL_YOUT_H 0x3D
#define MPU6000_ACCEL_YOUT_L 0x3E
#define MPU6000_ACCEL_ZOUT_H 0x3F
#define MPU6000_ACCEL_ZOUT_L 0x40
#define MPU6000_TEMP_OUT_H 0x41
#define MPU6000_TEMP_OUT_L 0x42
#define MPU6000_GYRO_XOUT_H 0x43
#define MPU6000_GYRO_XOUT_L 0x44
#define MPU6000_GYRO_YOUT_H 0x45
#define MPU6000_GYRO_YOUT_L 0x46
#define MPU6000_GYRO_ZOUT_H 0x47
#define MPU6000_GYRO_ZOUT_L 0x48
#define MPU6000_USER_CTRL 0x6A
#define MPU6000_SIGNAL_PATH_RESET 0x68
#define MPU6000_PWR_MGMT_1 0x6B
#define MPU6000_PWR_MGMT_2 0x6C
#define MPU6000_FIFO_COUNTH 0x72
#define MPU6000_FIFO_COUNTL 0x73
#define MPU6000_FIFO_R_W 0x74
#define MPU6000_WHOAMI 0x75

#define BIT_SLEEP 0x40
#define BIT_H_RESET 0x80
#define BITS_CLKSEL 0x07
#define MPU_CLK_SEL_PLLGYROX 0x01
#define MPU_CLK_SEL_PLLGYROZ 0x03
#define MPU_EXT_SYNC_GYROX 0x02
#define BITS_FS_250DPS 0x00
#define BITS_FS_500DPS 0x08
#define BITS_FS_1000DPS 0x10
#define BITS_FS_2000DPS 0x18
#define BITS_FS_2G 0x00
#define BITS_FS_4G 0x08
#define BITS_FS_8G 0x10
#define BITS_FS_16G 0x18
#define BITS_FS_MASK 0x18
#define BITS_DLPF_CFG_256HZ 0x00
#define BITS_DLPF_CFG_188HZ 0x01
#define BITS_DLPF_CFG_98HZ 0x02
#define BITS_DLPF_CFG_42HZ 0x03
#define BITS_DLPF_CFG_20HZ 0x04
#define BITS_DLPF_CFG_10HZ 0x05
#define BITS_DLPF_CFG_5HZ 0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF 0x07
#define BITS_DLPF_CFG_MASK 0x07
#define BIT_INT_ANYRD_2CLEAR 0x10
#define BIT_RAW_RDY_EN 0x01
#define BIT_I2C_IF_DIS 0x10
#define BIT_INT_STATUS_DATA 0x01
#define BIT_GYRO 3
#define BIT_ACC 2
#define BIT_TEMP 1

#define MPU6000_DEV_ID 0x68
#define MPU6000ES_REV_C4 0x14
#define MPU6000ES_REV_C5 0x15
#define MPU6000ES_REV_D6 0x16
#define MPU6000ES_REV_D7 0x17
#define MPU6000ES_REV_D8 0x18
#define MPU6000_REV_C4 0x54
#define MPU6000_REV_C5 0x55
#define MPU6000_REV_D6 0x56
#define MPU6000_REV_D7 0x57
#define MPU6000_REV_D8 0x58
#define MPU6000_REV_D9 0x59
#define MPU6000_REV_D10 0x5A

#define BITS_DLPF_CFG_256HZ 0x00
#define BITS_DLPF_CFG_188HZ 0x01
#define BITS_DLPF_CFG_98HZ 0x02
#define BITS_DLPF_CFG_42HZ 0x03

#define MPU_RF_DATA_RDY_EN (1 << 0)
#define MPU_CYCLE (1 << 5)

#define MPU6000_ACC_16G_SCALE 2048.0
#define MPU6000_ACC_8G_SCALE 4096.0
#define MPU6000_ACC_4G_SCALE 8192.0
#define MPU6000_ACC_2G_SCALE 16384.0

#define MPU6000_GYR_2000DPS_SCALE 16.4
#define MPU6000_GYR_1000DPS_SCALE 32.8
#define MPU6000_GYR_500DPS_SCALE 65.5
#define MPU6000_GYR_250DPS_SCALE 131.0

typedef enum
{
    MPU6000_No_Error = 0,
    MPU6000_Obj_Error,
    MPU6000_SampleRate_Error,
    MPU6000_BusCommunicate_Error,
    MPU6000_DevID_Error,
    MPU6000_SignalPathReset_Error,
    MPU6000_DisableI2C_Error,
    MPU6000_PWRMNG2_Set_Error,
    MPU6000_DIV_Set_Error,
    MPU6000_Gyr_Cfg_Error,
    MPU6000_Acc_Cfg_Error,
    MPU6000_IntPin_Set_Error,
    MPU6000_EnableInt_Error,
} DevMPU6000_Error_List;

typedef enum
{
    MPU6000_SampleRate_8K = 0,
    MPU6000_SampleRate_4K = 1,
    MPU6000_SampleRate_2K = 3,
    MPU6000_SampleRate_1K = 7,
} DevMPU6000_SampleRate_List;

typedef enum
{
    MPU6000_Gyr_250DPS = 0,
    MPU6000_Gyr_500DPS,
    MPU6000_Gyr_1000DPS,
    MPU6000_Gyr_2000DPS
} DevMPU6000_GyrTrip_List;

typedef enum
{
    MPU6000_Acc_2G = 0,
    MPU6000_Acc_4G,
    MPU6000_Acc_8G,
    MPU6000_Acc_16G,
} DevMPU6000_AccTrip_List;

typedef struct
{
    cs_ctl_callback cs_ctl;
    bus_trans_callback bus_trans;
    delay_callback delay;
    get_time_stamp_callback get_timestamp;

    uint8_t AccTrip;
    uint8_t GyrTrip;

    uint8_t PHY_AccTrip_Val;
    uint16_t PHY_GyrTrip_Val;

    float acc_scale;
    float gyr_scale;

    bool drdy;
    DevMPU6000_SampleRate_List rate;

    IMUData_TypeDef OriData;

    IMU_Error_TypeDef error;
} DevMPU6000Obj_TypeDef;

typedef struct
{
    bool (*detect)(bus_trans_callback trans, cs_ctl_callback cs_ctl);
    void (*pre_init)(DevMPU6000Obj_TypeDef *sensor_obj, cs_ctl_callback cs_ctl, bus_trans_callback bus_trans, delay_callback delay, get_time_stamp_callback ge_time_stamp);
    bool (*config)(DevMPU6000Obj_TypeDef *sensor_obj, DevMPU6000_SampleRate_List rate, DevMPU6000_AccTrip_List AccTrip, DevMPU6000_GyrTrip_List GyrTrip);
    bool (*init)(DevMPU6000Obj_TypeDef *sensor_obj);
    bool (*reset)(DevMPU6000Obj_TypeDef *snesor_obj);
    bool (*get_ready)(DevMPU6000Obj_TypeDef *sensor_obj);
    void (*set_ready)(DevMPU6000Obj_TypeDef *sensor_obj);
    bool (*sample)(DevMPU6000Obj_TypeDef *sensor_obj);
    IMUData_TypeDef (*get_data)(DevMPU6000Obj_TypeDef *sensor_obj);
    DevMPU6000_Error_List (*get_error)(DevMPU6000Obj_TypeDef *sensor_obj);
    IMU_Error_TypeDef (*get_scale)(const DevMPU6000Obj_TypeDef *sensor_obj);
    float (*get_gyr_angular_speed_diff)(const DevMPU6000Obj_TypeDef *sensor_obj);
} DevMPU6000_TypeDef;

extern DevMPU6000_TypeDef DevMPU6000;

#ifdef __cplusplus
}
#endif

#endif
