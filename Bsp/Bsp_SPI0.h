#ifndef __BSP_SPI_H
#define __BSP_SHI_H

#include <stdint.h>
#include <stdbool.h>
#include "Bsp_SPI_Cfg.h"

typedef struct
{
    bool (*Init)();
    void (*TransByte)(uint8_t tx);
    uint8_t (*ReceiveByte)(void);
    uint8_t (*TransMitByte)(uint8_t tx);
    uint16_t (*TransMitBuff)(uint8_t *tx, uint8_t *rx, uint16_t size);
} BspSpi_TypeDef;

#endif
