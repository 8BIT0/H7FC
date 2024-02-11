#ifndef __FCHW_CONFIG_H
#define __FCHW_CONFIG_H

#include "util.h"

#define ON 1
#define OFF 0

// #if !defined HW_BATEAIO_AT32F435 && !defined HW_MATEK_STM32H743
// #define HW_MATEK_STM32H743
// #endif

#if defined MATEKH743_V1_5

#define IMU_CNT 2
#define BARO_CNT 1
#define MAG_CNT 0
#define SD_CARD_ENABLE_STATE ON
#define FLASH_CHIP_ENABLE_STATE OFF
#define RADIO_NUM 1

#elif defined BATEAT32F435_AIO

#define IMU_CNT 1
#define BARO_CNT 1
#define MAG_CNT 0
#define SD_CARD_ENABLE_STATE OFF
#define FLASH_CHIP_ENABLE_STATE ON
#define RADIO_NUM 1

#endif

#define Flash_MaxRWSize (2 Kb)
#define Flash_Storage_TabSize (4 Kb)
#define Flash_Storage_InfoPageSize (1 Kb)

#define IMU_SUM IMU_CNT
#define BARO_SUM BARO_CNT
#define MAG_SUM MAG_CNT
#define SD_CARD SD_CARD_ENABLE_STATE
#define FLASH_CHIP_STATE FLASH_CHIP_ENABLE_STATE
#define RADIO_UART_NUM RADIO_NUM

#endif
