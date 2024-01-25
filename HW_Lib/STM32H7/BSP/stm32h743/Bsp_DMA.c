#include "Bsp_DMA_Port_Def.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc.h"
#include "Bsp_DMA.h"

static DMA_HandleTypeDef *BspDMA_Map[Bsp_DMA_Sum][Bsp_DMA_Stream_Sum] = {NULL};
static DMA_HandleTypeDef DataPipe_DMA;
static bool DataPipe_DMA_Init = false;

static const BspDMA1_Instance_List[Bsp_DMA_Stream_Sum] = {
    DMA1_Stream0,
    DMA1_Stream1,
    DMA1_Stream2,
    DMA1_Stream3,
    DMA1_Stream4,
    DMA1_Stream5,
    DMA1_Stream6,
    DMA1_Stream7};

static const BspDMA2_Instance_List[Bsp_DMA_Stream_Sum] = {
    DMA2_Stream0,
    DMA2_Stream1,
    DMA2_Stream2,
    DMA2_Stream3,
    DMA2_Stream4,
    DMA2_Stream5,
    DMA2_Stream6,
    DMA2_Stream7};

/* internal function */

/* external function */
static bool BspDMA_Regist_Obj(BspDMA_List dma, BspDMA_Stream_List stream, DMA_HandleTypeDef *hdl);
static bool BspDMA_Unregist_Obj(BspDMA_List dma, BspDMA_Stream_List stream);
static DMA_HandleTypeDef *BspDMA_Get_Handle(BspDMA_List dma, BspDMA_Stream_List stream);
static DMA2D_TypeDef *BspDMA_Get_Instance(BspDMA_List dma, BspDMA_Stream_List stream);
static void BspDMA_EnableIRQ(BspDMA_List dma, BspDMA_Stream_List stream, uint32_t preempt, uint32_t sub, uint32_t mux_seq, void *cb);

BspDMA_TypeDef BspDMA = {
    .regist = BspDMA_Regist_Obj,
    .unregist = BspDMA_Unregist_Obj,
    .get_handle = BspDMA_Get_Handle,
    .get_instance = BspDMA_Get_Instance,
    .enable_irq = BspDMA_EnableIRQ,
};

/* pipe external function */
static bool BspDMA_Pipe_Init(BspDMA_Pipe_TransFin_Cb fin_cb, BspDMA_Pipe_TransErr_Cb err_cb);
static bool BspDMA_Pipe_Trans(uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
static void* BspDMA_Get_Pipe_Handle(void);

BspDMA_Pipe_TypeDef BspDMA_Pipe = {
    .init = BspDMA_Pipe_Init,
    .trans = BspDMA_Pipe_Trans,
    .get_hanle = BspDMA_Get_Pipe_Handle,
};

/* DMA2_Stream7 for DataPipe Use */
static DMA2D_TypeDef *BspDMA_Get_Instance(BspDMA_List dma, BspDMA_Stream_List stream)
{
    static bool dma1_clk_init = false;
    static bool dma2_clk_init = false;

    if ((dma < Bsp_DMA_1) || (stream < Bsp_DMA_Stream_0))
        return NULL;

    if ((dma == Bsp_DMA_1) && ((stream < Bsp_DMA_Stream_Sum) && (stream >= Bsp_DMA_Stream_0)))
    {
        if (!dma1_clk_init)
        {
            __HAL_RCC_DMA1_CLK_ENABLE();
            dma1_clk_init = true;
        }

        return BspDMA1_Instance_List[stream];
    }
    else if ((dma == Bsp_DMA_2) && ((stream < Bsp_DMA_Stream_7) && (stream >= Bsp_DMA_Stream_0)))
    {
        if (!dma2_clk_init)
        {
            __HAL_RCC_DMA2_CLK_ENABLE();
            dma2_clk_init = true;
        }

        return BspDMA2_Instance_List[stream];
    }

    return NULL;
}

static bool BspDMA_Regist_Obj(BspDMA_List dma, BspDMA_Stream_List stream, DMA_HandleTypeDef *hdl)
{
    DMA2D_TypeDef *instance = NULL;

    if ((dma < Bsp_DMA_1) || (stream < Bsp_DMA_Stream_0))
        return false;

    if (((dma < Bsp_DMA_Sum) && (dma >= Bsp_DMA_1)) &&
        ((stream < Bsp_DMA_Stream_Sum) && (stream >= Bsp_DMA_Stream_0)))
    {
        instance = BspDMA_Get_Instance(dma, stream);
        if (instance == NULL)
            return false;

        hdl->Instance = instance;
        BspDMA_Map[dma][stream] = hdl;
        return true;
    }

    return false;
}

static bool BspDMA_Unregist_Obj(BspDMA_List dma, BspDMA_Stream_List stream)
{
    if ((dma < Bsp_DMA_1) || (stream < Bsp_DMA_Stream_0))
        return false;

    if (((dma < Bsp_DMA_Sum) && (dma >= Bsp_DMA_1)) &&
        ((stream < Bsp_DMA_Stream_Sum) && (stream >= Bsp_DMA_Stream_0)))
    {
        BspDMA_Map[dma][stream] = NULL;
        return true;
    }

    return false;
}

static DMA_HandleTypeDef *BspDMA_Get_Handle(BspDMA_List dma, BspDMA_Stream_List stream)
{
    if ((dma < Bsp_DMA_1) || (stream < Bsp_DMA_Stream_0))
        return NULL;

    if (((dma < Bsp_DMA_Sum) && (dma >= Bsp_DMA_1)) &&
        ((stream < Bsp_DMA_Stream_Sum) && (stream >= Bsp_DMA_Stream_0)))
    {
        return BspDMA_Map[dma][stream];
    }

    return NULL;
}

static void BspDMA_EnableIRQ(BspDMA_List dma, BspDMA_Stream_List stream, uint32_t preempt, uint32_t sub, uint32_t mux_seq, void *cb)
{
    UNUSED(mux_seq);
    UNUSED(cb);

    IRQn_Type irq;

    if ((dma < Bsp_DMA_1) || (stream < Bsp_DMA_Stream_0))
        return;

    if (dma == Bsp_DMA_1)
    {
        switch (stream)
        {
        case Bsp_DMA_Stream_0:
            irq = DMA1_Stream0_IRQn;
            break;

        case Bsp_DMA_Stream_1:
            irq = DMA1_Stream1_IRQn;
            break;

        case Bsp_DMA_Stream_2:
            irq = DMA1_Stream2_IRQn;
            break;

        case Bsp_DMA_Stream_3:
            irq = DMA1_Stream3_IRQn;
            break;

        case Bsp_DMA_Stream_4:
            irq = DMA1_Stream4_IRQn;
            break;

        case Bsp_DMA_Stream_5:
            irq = DMA1_Stream5_IRQn;
            break;

        case Bsp_DMA_Stream_6:
            irq = DMA1_Stream6_IRQn;
            break;

        case Bsp_DMA_Stream_7:
            irq = DMA1_Stream7_IRQn;
            break;

        default:
            return;
        }
    }
    else if (dma == Bsp_DMA_2)
    {
        switch (stream)
        {
        case Bsp_DMA_Stream_0:
            irq = DMA2_Stream0_IRQn;
            break;

        case Bsp_DMA_Stream_1:
            irq = DMA2_Stream1_IRQn;
            break;

        case Bsp_DMA_Stream_2:
            irq = DMA2_Stream2_IRQn;
            break;

        case Bsp_DMA_Stream_3:
            irq = DMA2_Stream3_IRQn;
            break;

        case Bsp_DMA_Stream_4:
            irq = DMA2_Stream4_IRQn;
            break;

        case Bsp_DMA_Stream_5:
            irq = DMA2_Stream5_IRQn;
            break;

        case Bsp_DMA_Stream_6:
            irq = DMA2_Stream6_IRQn;
            break;

        case Bsp_DMA_Stream_7:
            irq = DMA2_Stream7_IRQn;
            break;

        default:
            return;
        }
    }
    else
        return;

    HAL_NVIC_SetPriority(irq, preempt, sub);
    HAL_NVIC_EnableIRQ(irq);
}

static bool BspDMA_Pipe_Init(BspDMA_Pipe_TransFin_Cb fin_cb, BspDMA_Pipe_TransErr_Cb err_cb)
{
    if(fin_cb == NULL)
        return false;

    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* Configure DMA request DataPipe_DMA on DMA2_Stream7 */
    DataPipe_DMA.Instance = DMA2_Stream7;
    DataPipe_DMA.Init.Request = DMA_REQUEST_MEM2MEM;
    DataPipe_DMA.Init.Direction = DMA_MEMORY_TO_MEMORY;
    DataPipe_DMA.Init.PeriphInc = DMA_PINC_ENABLE;
    DataPipe_DMA.Init.MemInc = DMA_MINC_ENABLE;
    DataPipe_DMA.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    DataPipe_DMA.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    DataPipe_DMA.Init.Mode = DMA_NORMAL;
    DataPipe_DMA.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    DataPipe_DMA.Init.FIFOMode = DMA_FIFOMODE_ENABLE;

    if (HAL_DMA_Init(&DataPipe_DMA) != HAL_OK)
        return false;

    /* set transmit process callback */
    HAL_DMA_RegisterCallback(&DataPipe_DMA, HAL_DMA_XFER_CPLT_CB_ID, fin_cb);
    HAL_DMA_RegisterCallback(&DataPipe_DMA, HAL_DMA_XFER_ERROR_CB_ID, err_cb);

    /* DMA interrupt init */
    /* DMA2_Stream7_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

    DataPipe_DMA_Init = true;

    return true;
}

static bool BspDMA_Pipe_Trans(uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
    if (!DataPipe_DMA_Init || \
        (SrcAddress == 0) || \
        (DstAddress == 0) || \
        (DataLength == 0))
        return false;

    if(HAL_DMA_Start_IT(&DataPipe_DMA, SrcAddress, DstAddress, DataLength) != HAL_OK)
    {
        HAL_DMA_Abort_IT(&DataPipe_DMA);
        return false;
    }

    return true;
}

static void* BspDMA_Get_Pipe_Handle(void)
{
    if(!DataPipe_DMA_Init)
        return NULL;

    return (void *)&DataPipe_DMA;
}
