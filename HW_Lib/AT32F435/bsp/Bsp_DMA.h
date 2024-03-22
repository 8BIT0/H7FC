#ifndef __BSP_DMA_H
#define __BSP_DMA_H

#include "Bsp_DMA_Port_Def.h"
#include "at32f435_437_dma.h"

typedef struct
{
    void (*BspDMA_Irq_Callback_Func)(void *cus);
    void *cus_data;
}BspDMA_IrqCall_Obj_TypeDef;

#define To_DMA_Handle_Ptr(x) ((dma_channel_type *)x)
#define To_DMA_IrqCallbackObj_Ptr(x) ((BspDMA_IrqCall_Obj_TypeDef *)x)

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
void BspDMA_Irq_Callback(void *channel);

extern BspDMA_TypeDef BspDMA;
extern BspDMA_Pipe_TypeDef BspDMA_Pipe;

#endif
