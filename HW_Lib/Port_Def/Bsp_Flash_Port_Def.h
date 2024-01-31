#ifndef __BSP_FLASH_PORT_DEF_H
#define __BSP_FLASH_PORT_DEF_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct
{
    bool (*init)(void);
    void (*de_init)(void);
    bool (*erase)(uint32_t addr, uint32_t len);
    uint8_t (*get_align_size)(void);
    bool (*read)(uint32_t addr, uint8_t *p_data, uint32_t size);
    bool (*write)(uint32_t addr, uint8_t *p_data, uint32_t size);
} BspFlash_TypeDef;

#endif
