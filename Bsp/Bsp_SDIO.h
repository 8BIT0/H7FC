#ifndef __BSP_SDIO_H
#define __BSP_SDIO_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define SDIO_USE_4BIT true
#define SDIO_CK_PIN PC12
#define SDIO_CMD_PIN PD2
#define SDIO_D0_PIN PC8
#define SDIO_D1_PIN PC9
#define SDIO_D2_PIN PC10
#define SDIO_D3_PIN PC11

typedef struct
{
    bool (*init)(void);
    bool (*read)(uint32_t addr, uint8_t *data, uint32_t size);
    bool (*write)(uint32_t addr, uint8_t *data, uint32_t size);
} BspSDIO_TypeDef;

#endif
