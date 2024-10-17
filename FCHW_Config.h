#ifndef __FCHW_CONFIG_H
#define __FCHW_CONFIG_H

#include "util.h"

#define ON 1
#define OFF 0

// #if !defined HW_BATEAIO_AT32F435 && !defined HW_MATEK_STM32H743
// #define HW_MATEK_STM32H743
// #endif

#define Storage_ChipBus_None    0
#define Storage_ChipBus_Spi     (Storage_ChipBus_None + 1)
#define Storage_ChipBus_QSpi    (Storage_ChipBus_Spi + 1)

#if defined MATEKH743_V1_5

#define IMU_CNT                 2
#define BARO_CNT                1
#define MAG_CNT                 0
#define SD_CARD_ENABLE_STATE    ON
#define SDRAM_ENABLE_STATE      OFF
#define FLASH_CHIP_ENABLE_STATE OFF
#define RADIO_NUM               1

#define Flash_MaxRWSize (0 Kb)
#define Flash_Storage_TabSize (0 Kb)
#define Flash_Storage_InfoPageSize (0 Kb)

#elif defined NEURE 

#define IMU_CNT                 2
#define BARO_CNT                1
#define MAG_CNT                 1
#define SD_CARD_ENABLE_STATE    OFF
#define SDRAM_ENABLE_STATE      ON
#define FLASH_CHIP_ENABLE_STATE OFF
#define RADIO_NUM               1

#if (SDRAM_ENABLE_STATE == ON)
#define SDRAM_BASE_ADDR         (32 Mb)   
#define SDRAN_MEM_SIZE          ((uint32_t)0xC0000000)
#endif

#define Flash_MaxRWSize (2 Kb)
#define Flash_Storage_TabSize (4 Kb)
#define Flash_Storage_InfoPageSize (1 Kb)

#elif defined BATEAT32F435_AIO || defined CCRC_AT32_20 || defined CAIFPV_AIO

#define IMU_CNT                 1
#define BARO_CNT                1
#define MAG_CNT                 0
#define SD_CARD_ENABLE_STATE    OFF
#define SDRAM_ENABLE_STATE      OFF
#define FLASH_CHIP_ENABLE_STATE ON
#define RADIO_NUM               1

#define Flash_MaxRWSize (2 Kb)
#define Flash_Storage_TabSize (4 Kb)
#define Flash_Storage_InfoPageSize (1 Kb)

#endif

#define Storage_InfoPageSize Flash_Storage_InfoPageSize

/* get virable from .ld file defined */
extern uint32_t __rom_s;
extern uint32_t __rom_e;
extern uint32_t __boot_s;
extern uint32_t __boot_e;

#define Boot_Address_Base   ((uint32_t)(&__boot_s))
#define Boot_Section_Size   ((uint32_t)&__boot_e - (uint32_t)&__boot_s)

#define App_Address_Base    ((uint32_t)&__boot_e)
#define App_Section_Size    ((uint32_t)&__rom_e - App_Address_Base)

#define IMU_SUM             IMU_CNT
#define BARO_SUM            BARO_CNT
#define MAG_SUM             MAG_CNT
#define SDRAM_EN            SDRAM_ENABLE_STATE
#define SD_CARD             SD_CARD_ENABLE_STATE
#define FLASH_CHIP_STATE    FLASH_CHIP_ENABLE_STATE
#define RADIO_UART_NUM      RADIO_NUM
#if (SDRAM_EN == ON)
#define FC_SDRAM_Size       SDRAM_BASE_ADDR
#define FC_SDRAM_Base_Addr  SDRAN_MEM_SIZE
#endif

#endif
