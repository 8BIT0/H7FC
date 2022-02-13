#ifndef __BSP_SPI_H
#define __BSP_SHI_H

#include <stdint.h>
#include <stdbool.h>
#include "Bsp_SPI_Cfg.h"

typedef enum
{
    BspSPI_Norm_Mode = 0,
    BspSPI_Quad_Mode,
} BspSPI_Mode_List;

#pragma pack(1)
typedef struct
{
    uint8_t id;
} BspSpiNormalModeObj_TypeDef;

typedef struct
{
    uint8_t id;
} BspSpiQuadModeObj_TypeDef;
#pragma pack()

typedef struct
{
    bool (*Init)();
    void (*TransByte)(uint8_t tx);
    uint8_t (*ReceiveByte)(void);
    uint8_t (*TransMitByte)(uint8_t tx);
    uint16_t (*TransMitBuff)(uint8_t *tx, uint8_t *rx, uint16_t size);
} BspSpi_TypeDef;

#endif
