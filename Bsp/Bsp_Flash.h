#ifndef __BSP_FLASH_H
#define __BSP_FLASH_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct
{
    bool (*init)(void);
    void (*de_init)(void);
    bool (*erase_sector)(uint32_t addr);
    bool (*read)(uint32_t addr, uint8_t *p_data, uint16_t size);
    bool (*write)(uint32_t addr, uint8_t *p_data, uint16_t size);
} BspFlash_TypeDef;

extern BspFlash_TypeDef BspFlash;

#endif
