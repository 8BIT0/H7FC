#ifndef __BSP_DMA_H
#define __BSP_DMA_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal_dma.h"

typedef enum
{
    Bsp_DMA_1 = 0,
    Bsp_DMA_2,
    Bsp_DMA_Sum,
} BspDMA_List;

typedef enum
{
    Bsp_DMA_Stream_0 = 0,
    Bsp_DMA_Stream_1,
    Bsp_DMA_Stream_2,
    Bsp_DMA_Stream_3,
    Bsp_DMA_Stream_4,
    Bsp_DMA_Stream_5,
    Bsp_DMA_Stream_6,
    Bsp_DMA_Stream_7,
    Bsp_DMA_Stream_Sum,
} BspDMA_Stream_List;

typedef struct
{
    bool (*regist)(BspDMA_List dma, BspDMA_Stream_List stream, DMA_HandleTypeDef *obj);
    bool (*unregist)(BspDMA_List dma, BspDMA_Stream_List stream);
    DMA_HandleTypeDef *(*get_handle)(BspDMA_List dma, BspDMA_Stream_List stream);
    DMA2D_TypeDef *(*get_instance)(BspDMA_List dma, BspDMA_Stream_List stream);
} BspDMA_TypeDef;

extern BspDMA_TypeDef BspDMA;

#endif
