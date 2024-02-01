#ifndef __FCHW_CONFIG_H
#define __FCHW_CONFIG_H

#define ON 1
#define OFF 0

#if !defined HW_BATEAIO_AT32F435 && !defined HW_MATEK_STM32H743
#define HW_MATEK_STM32H743
#endif

#if defined HW_MATEK_STM32H743

#define IMU_CNT 2
#define BARO_CNT 1
#define MAG_CNT 0
#define SD_CARD_ENABLE_STATE ON
#define FLASH_CHIP_ENABLE_STATE OFF

#elif defined HW_BATEAIO_AT32F435

#define AT32F435RGT7

#define IMU_CNT 1
#define BARO_CNT 1
#define MAG_CNT 0
#define SD_CARD_ENABLE_STATE OFF
#define FLASH_CHIP_ENABLE_STATE ON

#else

#ifndef IMU_CNT
#define IMU_CNT 1
#endif

#ifndef BARO_CNT
#define BARO_CNT 1
#endif

#ifndef MAG_CNT
#define MAG_CNT 0
#endif

#ifndef SD_CARD_ENABLE_STATE
#define SD_CARD_ENABLE_STATE OFF
#endif

#ifndef FLASH_CHIP_ENABLE_STATE
#define FLASH_CHIP_ENABLE_STATE OFF
#endif

#endif

#define IMU_SUM IMU_CNT
#define BARO_SUM BARO_CNT
#define MAG_SUM MAG_CNT
#define SD_CARD SD_CARD_ENABLE_STATE
#define FLASH_CHIP FLASH_CHIP_ENABLE_STATE

#endif
