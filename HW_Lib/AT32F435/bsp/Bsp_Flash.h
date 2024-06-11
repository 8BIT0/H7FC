#ifndef __BSP_FLASH_H
#define __BSP_FLASH_H

#include "Bsp_Flash_Port_Def.h"

#define FLASH_DEFAULT_DATA 0xFF

#define FLASH_BANK_NUM 2
#define FLASH_BANK_SIZE (512 * 1024)
#define FLASH_BLOCK_PER_BANK 8
#define FLASH_SECTION_PER_BLOCK 32

#define FLASH_ALIGN_SIZE 4

#define FLASH_SECTION_SIZE (2 * 1024)

#define FLASH_BLOCK_END_ADDR 0x080FA000

#define FLASH_BLOCK_END

extern BspFlash_TypeDef BspFlash;

#endif

