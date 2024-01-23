#ifndef __BSP_DMA_H
#define __BSP_DMA_H

#include "Bsp_DMA_Port_Def.h"

typedef enum
{
    Bsp_DMA_None = -1,
    Bsp_DMA_1 = 0,
    Bsp_DMA_2,
    Bsp_DMA_Sum,
} BspDMA_List;

typedef enum
{
    Bsp_DMA_Stream_None = -1,
    Bsp_DMA_Stream_1 = 0,
    Bsp_DMA_Stream_2,
    Bsp_DMA_Stream_3,
    Bsp_DMA_Stream_4,
    Bsp_DMA_Stream_5,
    Bsp_DMA_Stream_6,
    Bsp_DMA_Stream_7,
    Bsp_DMA_Stream_Sum,
} BspDMA_Stream_List;

void BspDMA_Pipe_Irq_Callback(void);

extern BspDMA_TypeDef BspDMA;
extern BspDMA_Pipe_TypeDef BspDMA_Pipe;

#endif
