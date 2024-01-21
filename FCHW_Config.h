#ifndef __FCHW_CONFIG_H
#define __FCHW_CONFIG_H

#define ENABLE 1
#define DISABLE 0

#if defined HW_MATEK_STM32H743

#define BARO_NUM 1
#define MAG_NUM 0
#define SD_CARD ENABLE
#define FLASH_CHIP DISABLE

#elif defined HW_BATEAIO_AT32F435

#define AT32F435RGT7

#define IMU_SUM 1
#define BARO_NUM 1
#define MAG_NUM 0
#define SD_CARD DISABLE
#define FLASH_CHIP ENABLE

#else

#define IMU_SUM 1
#define SD_CARD DISABLE
#define FLASH_CHIP DISABLE

#endif

#endif
