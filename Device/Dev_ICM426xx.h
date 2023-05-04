#ifndef __DEV_ICM426XX_H
#define __DEV_ICM426XX_H

#include <stdbool.h>
#include <stdint.h>
#include "imu_data.h"

// 24 MHz max SPI frequency
#define ICM426XX_MAX_SPI_CLK_HZ 24000000

#define ICM42605_DEV_ID 0x42
#define ICM42688P_DEV_ID 0x47

#define ICM426XX_WHO_AM_I 0x75
#define ICM426XX_READ_MASK 0x80

#define ICM426XX_RA_REG_BANK_SEL 0x76
#define ICM426XX_BANK_SELECT0 0x00
#define ICM426XX_BANK_SELECT1 0x01
#define ICM426XX_BANK_SELECT2 0x02
#define ICM426XX_BANK_SELECT3 0x03
#define ICM426XX_BANK_SELECT4 0x04

#define ICM426XX_RA_PWR_MGMT0 0x4E // User Bank 0
#define ICM426XX_PWR_MGMT0_ACCEL_MODE_LN (3 << 0)
#define ICM426XX_PWR_MGMT0_GYRO_MODE_LN (3 << 2)
#define ICM426XX_PWR_MGMT0_GYRO_ACCEL_MODE_OFF ((0 << 0) | (0 << 2))
#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF (0 << 5)
#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_ON (1 << 5)

#define ICM426XX_RA_GYRO_CONFIG0 0x4F
#define ICM426XX_RA_ACCEL_CONFIG0 0x50

// --- Registers for gyro and acc Anti-Alias Filter ---------
#define ICM426XX_RA_GYRO_CONFIG_STATIC3 0x0C  // User Bank 1
#define ICM426XX_RA_GYRO_CONFIG_STATIC4 0x0D  // User Bank 1
#define ICM426XX_RA_GYRO_CONFIG_STATIC5 0x0E  // User Bank 1
#define ICM426XX_RA_ACCEL_CONFIG_STATIC2 0x03 // User Bank 2
#define ICM426XX_RA_ACCEL_CONFIG_STATIC3 0x04 // User Bank 2
#define ICM426XX_RA_ACCEL_CONFIG_STATIC4 0x05 // User Bank 2
// --- Register & setting for gyro and acc UI Filter --------
#define ICM426XX_RA_GYRO_ACCEL_CONFIG0 0x52 // User Bank 0
#define ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY (15 << 4)
#define ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY (15 << 0)
// ----------------------------------------------------------

#define ICM426XX_RA_GYRO_DATA_X1 0x25  // User Bank 0
#define ICM426XX_RA_ACCEL_DATA_X1 0x1F // User Bank 0

#define ICM426XX_RA_INT_CONFIG 0x14 // User Bank 0
#define ICM426XX_INT1_MODE_PULSED (0 << 2)
#define ICM426XX_INT1_MODE_LATCHED (1 << 2)
#define ICM426XX_INT1_DRIVE_CIRCUIT_OD (0 << 1)
#define ICM426XX_INT1_DRIVE_CIRCUIT_PP (1 << 1)
#define ICM426XX_INT1_POLARITY_ACTIVE_LOW (0 << 0)
#define ICM426XX_INT1_POLARITY_ACTIVE_HIGH (1 << 0)

#define ICM426XX_RA_INT_CONFIG0 0x63 // User Bank 0
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR ((0 << 5) || (0 << 4))
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR_DUPLICATE ((0 << 5) || (0 << 4)) // duplicate settings in datasheet, Rev 1.2.
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_F1BR ((1 << 5) || (0 << 4))
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR_AND_F1BR ((1 << 5) || (1 << 4))

#define ICM426XX_RA_INT_CONFIG1 0x64 // User Bank 0
#define ICM426XX_INT_ASYNC_RESET_BIT 4
#define ICM426XX_INT_TDEASSERT_DISABLE_BIT 5
#define ICM426XX_INT_TDEASSERT_ENABLED (0 << ICM426XX_INT_TDEASSERT_DISABLE_BIT)
#define ICM426XX_INT_TDEASSERT_DISABLED (1 << ICM426XX_INT_TDEASSERT_DISABLE_BIT)
#define ICM426XX_INT_TPULSE_DURATION_BIT 6
#define ICM426XX_INT_TPULSE_DURATION_100 (0 << ICM426XX_INT_TPULSE_DURATION_BIT)
#define ICM426XX_INT_TPULSE_DURATION_8 (1 << ICM426XX_INT_TPULSE_DURATION_BIT)

#define ICM426XX_RA_INT_SOURCE0 0x65 // User Bank 0
#define ICM426XX_UI_DRDY_INT1_EN_DISABLED (0 << 3)
#define ICM426XX_UI_DRDY_INT1_EN_ENABLED (1 << 3)

#define ICM426XX_ACC_16G_SCALE 2048.0
#define ICM426XX_ACC_8G_SCALE 4096.0
#define ICM426XX_ACC_4G_SCALE 8192.0
#define ICM426XX_ACC_2G_SCALE 16384.0

#define ICM426XX_GYR_2000DPS_SCALE 16.4
#define ICM426XX_GYR_1000DPS_SCALE 32.8
#define ICM426XX_GYR_500DPS_SCALE 65.5
#define ICM426XX_GYR_250DPS_SCALE 131.0

typedef enum
{
    ICM42688P = 0,
    ICM42605,
    ICM_NONE,
} ICM426xx_Sensor_TypeList;

typedef enum
{
    ICM426xx_No_Error = 0,
    ICM426xx_Obj_Error,
    ICM426xx_SampleRate_Error,
    ICM426xx_BusCommunicate_Error,
    ICM426xx_DevID_Error,
    ICM426xx_AccRangeOdr_Set_Error,
    ICM426xx_GyrRangeOdr_Set_Error,
    ICM426xx_AccGyr_TurnOff_Error,
    ICM426xx_AccGyr_TurnOn_Error,
} ICM426xx_Error_List;

typedef enum
{
    ICM426xx_SampleRate_8K = 0,
    ICM426xx_SampleRate_4K,
    ICM426xx_SampleRate_2K,
    ICM426xx_SampleRate_1K,
    ICM426xx_SampleRate_Sum,
} ICM426xx_SampleRate_List;

typedef enum
{
    ICM426xx_AAF_256Hz = 0,
    ICM426xx_AAF_536Hz,
    ICM426xx_AAF_997Hz,
    ICM426xx_AAF_1962Hz,
    ICM426xx_AAF_Sum,
} ICM426xx_AAF_List;

typedef struct
{
    uint8_t delt;
    uint16_t deltSqr;
    uint8_t bitshif;
} ICM426xx_AAF_Config_TypeDef;

typedef enum
{
    ICM426xx_ODR_8K = 3,
    ICM426xx_ODR_4K,
    ICM426xx_ODR_2K,
    ICM426xx_ODR_1K,
} ICM426xx_ConfigODR_ValList;

typedef enum
{
    ICM426xx_Gyr_250DPS = 0,
    ICM426xx_Gyr_500DPS,
    ICM426xx_Gyr_1000DPS,
    ICM426xx_Gyr_2000DPS
} ICM426xx_GyrTrip_List;

typedef enum
{
    ICM426xx_Acc_2G = 0,
    ICM426xx_Acc_4G,
    ICM426xx_Acc_8G,
    ICM426xx_Acc_16G,
} ICM426xx_AccTrip_List;

typedef struct
{
    ICM426xx_Sensor_TypeList type;

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

    ICM426xx_Error_List error;
    ICM426xx_SampleRate_List rate;

    IMUData_TypeDef OriData;
} DevICM426xxObj_TypeDef;

typedef struct
{
    ICM426xx_Sensor_TypeList (*detect)(bus_trans_callback trans, cs_ctl_callback cs_ctl);
    bool (*config)(DevICM426xxObj_TypeDef *Obj, ICM426xx_SampleRate_List rate, ICM426xx_GyrTrip_List GyrTrip, ICM426xx_AccTrip_List AccTrip);
    void (*pre_init)(DevICM426xxObj_TypeDef *Obj, cs_ctl_callback cs_ctl, bus_trans_callback bus_trans, delay_callback delay, get_time_stamp_callback get_time_stamp);
    bool (*init)(DevICM426xxObj_TypeDef *Obj);
    void (*set_ready)(DevICM426xxObj_TypeDef *Obj);
    bool (*get_ready)(DevICM426xxObj_TypeDef *Obj);
    bool (*reset)(DevICM426xxObj_TypeDef *Obj);
    bool (*sample)(DevICM426xxObj_TypeDef *Obj);
} DevICM426xx_TypeDef;

extern DevICM426xx_TypeDef DevICM426xx;

#endif
