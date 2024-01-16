#ifndef __BSP_FLASH_H
#define __BSP_FLASH_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "Bsp_Flash_Port_Def.h"

#define FLASH_DEFAULT_DATA              0xFF

#define FLASH_BASE_ADDR                 FLASH_BANK1_BASE

#define FLASH_SECTOR_0_OFFSET_ADDR      ((uint32_t)0x00000000)                  //Base address of Sector 0, 128 Kbytes
#define FLASH_SECTOR_1_OFFSET_ADDR      ((uint32_t)0x00020000)                  //Base address of Sector 1, 128 Kbytes
#define FLASH_SECTOR_2_OFFSET_ADDR      ((uint32_t)0x00040000)                  //Base address of Sector 2, 128 Kbytes
#define FLASH_SECTOR_3_OFFSET_ADDR      ((uint32_t)0x00060000)                  //Base address of Sector 3, 128 Kbytes
#define FLASH_SECTOR_4_OFFSET_ADDR      ((uint32_t)0x00080000)                  //Base address of Sector 4, 128 Kbytes
#define FLASH_SECTOR_5_OFFSET_ADDR      ((uint32_t)0x000A0000)                  //Base address of Sector 5, 128 Kbytes
#define FLASH_SECTOR_6_OFFSET_ADDR      ((uint32_t)0x000C0000)                  //Base address of Sector 6, 128 Kbytes
#define FLASH_SECTOR_7_OFFSET_ADDR      ((uint32_t)0x000E0000)                  //Base address of Sector 7, 128 Kbytes

#define FLASH_SECTOR_0_SIZE             ((uint32_t)0x00020000)
#define FLASH_SECTOR_1_SIZE             ((uint32_t)0x00020000)
#define FLASH_SECTOR_2_SIZE             ((uint32_t)0x00020000)
#define FLASH_SECTOR_3_SIZE             ((uint32_t)0x00020000)
#define FLASH_SECTOR_4_SIZE             ((uint32_t)0x00020000)
#define FLASH_SECTOR_5_SIZE             ((uint32_t)0x00020000)
#define FLASH_SECTOR_6_SIZE             ((uint32_t)0x00020000)
#define FLASH_SECTOR_7_SIZE             ((uint32_t)0x00020000)

extern BspFlash_TypeDef BspFlash;

#endif
