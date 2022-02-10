#ifndef __BSP_SPI_CFG_H
#define __BSP_SPI_CFG_H

#include <stdbool.h>
#include <stdint.h>

#pragma pack(1)
typedef struct
{
    uint32_t mode;
    uint32_t dirction;
    uint32_t data_size;
    uint32_t CLKPolarity;
    uint32_t CLKPhase;
    uint32_t PreScaler;
} BspSPI_GenModeCfg_TypeDef;
#pragma pack()

#endif
