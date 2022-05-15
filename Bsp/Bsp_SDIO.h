#ifndef __BSP_SDIO_H
#define __BSP_SDIO_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef struct
{
    bool (*init)(void);
    bool (*read)(uint32_t addr, uint8_t *data, uint32_t size);
    bool (*write)(uint32_t addr, uint8_t *data, uint32_t size);
} BspSDIO_TypeDef;

#endif
