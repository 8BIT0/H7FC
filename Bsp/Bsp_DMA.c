#include "Bsp_DMA.h"
#include "stm32h743xx.h"

static DMA_HandleTypeDef *BspDMA_Map[Bsp_DMA_Sum][Bsp_DMA_Stream_Sum] = {NULL};

static const BspDMA1_Instance_List[Bsp_DMA_Stream_Sum] = {
    DMA1_Stream0,
    DMA1_Stream1,
    DMA1_Stream2,
    DMA1_Stream3,
    DMA1_Stream4,
    DMA1_Stream5,
    DMA1_Stream6,
    DMA1_Stream7
};

static const BspDMA2_Instance_List[Bsp_DMA_Stream_Sum] = {
    DMA2_Stream0,
    DMA2_Stream1,
    DMA2_Stream2,
    DMA2_Stream3,
    DMA2_Stream4,
    DMA2_Stream5,
    DMA2_Stream6,
    DMA2_Stream7
};

/* internal function */

/* external function */
static bool BspDMA_Regist_Obj(BspDMA_List dma, BspDMA_Stream_List stream, DMA_HandleTypeDef *hdl);
static bool BspDMA_Unregist_Obj(BspDMA_List dma, BspDMA_Stream_List stream);
static DMA_HandleTypeDef *BspDMA_Get_Handle(BspDMA_List dma, BspDMA_Stream_List stream);

BspDMA_TypeDef BspDMA = {
    .regist = BspDMA_Regist_Obj,
    .unregist = BspDMA_Unregist_Obj,
    .get_handle = BspDMA_Get_Handle,
};

static DMA2D_TypeDef *BspDMA_Get_Instance(BspDMA_List dma, BspDMA_Stream_List stream)
{
    DMA2D_TypeDef *instance;

    if((dma == Bsp_DMA_1) && ((stream < Bsp_DMA_Stream_Sum) && (stream >= Bsp_DMA_Stream_0)))
    {
        return BspDMA1_Instance_List[stream];
    }
    else if((dma == Bsp_DMA_2) && ((stream < Bsp_DMA_Stream_Sum) && (stream >= Bsp_DMA_Stream_0)))
    {
        return BspDMA2_Instance_List[stream];
    }

    return NULL;
}

static bool BspDMA_Regist_Obj(BspDMA_List dma, BspDMA_Stream_List stream, DMA_HandleTypeDef *hdl)
{
    DMA2D_TypeDef *instance = NULL;

    if( ((dma < Bsp_DMA_Sum) && (dma >= Bsp_DMA_1)) && 
        ((stream < Bsp_DMA_Stream_Sum) && (stream >= Bsp_DMA_Stream_0)))
    {
        instance = BspDMA_Get_Instance(dma, stream);
        if(instance == NULL)
            return false;

        hdl->Instance = instance;
        BspDMA_Map[dma][stream] = hdl;
        return true;
    }

    return false;
}

static bool BspDMA_Unregist_Obj(BspDMA_List dma, BspDMA_Stream_List stream)
{
    if( ((dma < Bsp_DMA_Sum) && (dma >= Bsp_DMA_1)) && 
        ((stream < Bsp_DMA_Stream_Sum) && (stream >= Bsp_DMA_Stream_0)))
    {
        BspDMA_Map[dma][stream] = NULL;
        return true;
    }

    return false;
}

static DMA_HandleTypeDef *BspDMA_Get_Handle(BspDMA_List dma, BspDMA_Stream_List stream)
{
    if( ((dma < Bsp_DMA_Sum) && (dma >= Bsp_DMA_1)) && 
        ((stream < Bsp_DMA_Stream_Sum) && (stream >= Bsp_DMA_Stream_0)))
    {
        return BspDMA_Map[dma][stream];
    }

    return NULL;
}
