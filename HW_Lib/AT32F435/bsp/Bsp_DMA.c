#include "Bsp_DMA.h"
#include "at32f435_437_dma.h"

static const dma_channel_type* BspDMA1_Instance_List[Bsp_DMA_Stream_Sum] = {
    DMA1_CHANNEL1,
    DMA1_CHANNEL2,
    DMA1_CHANNEL3,
    DMA1_CHANNEL4,
    DMA1_CHANNEL5,
    DMA1_CHANNEL6,
    DMA1_CHANNEL7};

static const dma_channel_type* BspDMA2_Instance_List[Bsp_DMA_Stream_Sum] = {
    DMA2_CHANNEL1,
    DMA2_CHANNEL2,
    DMA2_CHANNEL3,
    DMA2_CHANNEL4,
    DMA2_CHANNEL5,
    DMA2_CHANNEL6,
    DMA2_CHANNEL7};

/* internal function */
static bool DataPipe_DMA_Init = false;
static bool DataPipe_InTrans = false;
static BspDMA_Pipe_TransFin_Cb DataPipe_Trans_Fin_Callback = NULL;
static BspDMA_Pipe_TransErr_Cb DataPipe_Trans_Err_Callback = NULL;

/* external funtion */
static bool BspDMA_Regist_Obj(BspDMA_List dma, BspDMA_Stream_List stream, void *hdl);
static bool BspDMA_Unregist_Obj(BspDMA_List dma, BspDMA_Stream_List stream);
static void *BspDMA_Get_Handle(BspDMA_List dma, BspDMA_Stream_List stream);
static dma_channel_type *BspDMA_Get_Instance(BspDMA_List dma, BspDMA_Stream_List stream);
static void BspDMA_EnableIRQ(BspDMA_List dma, BspDMA_Stream_List stream, uint32_t preempt, uint32_t sub);

/* pipe external function */
static bool BspDMA_Pipe_Init(BspDMA_Pipe_TransFin_Cb fin_cb, BspDMA_Pipe_TransErr_Cb err_cb);
static bool BspDMA_Pipe_Trans(uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
static void *BspDMA_Get_Pipe_Handle(void);

BspDMA_TypeDef BspDMA = {
    .regist = BspDMA_Regist_Obj,
    .unregist = BspDMA_Unregist_Obj,
    .get_handle = BspDMA_Get_Handle,
    .get_instance = BspDMA_Get_Instance,
    .enable_irq = BspDMA_EnableIRQ,
};

BspDMA_Pipe_TypeDef BspDMA_Pipe = {
    .init = BspDMA_Pipe_Init,
    .trans = BspDMA_Pipe_Trans,
    .get_hanle = BspDMA_Get_Pipe_Handle,
};

static bool BspDMA_Regist_Obj(BspDMA_List dma, BspDMA_Stream_List stream, void *hdl)
{
    UNUSED(dma);
    UNUSED(stream);
    UNUSED(hdl);

    return true;
}

static bool BspDMA_Unregist_Obj(BspDMA_List dma, BspDMA_Stream_List stream)
{
    UNUSED(dma);
    UNUSED(stream);
    
    return true;
}

static void *BspDMA_Get_Handle(BspDMA_List dma, BspDMA_Stream_List stream)
{
    UNUSED(dma);
    UNUSED(stream);

    return NULL;
}

static dma_channel_type *BspDMA_Get_Instance(BspDMA_List dma, BspDMA_Stream_List stream)
{
    static bool dma1_clk_init = false;
    static bool dma2_clk_init = false;

    if ((dma < Bsp_DMA_1) || (stream < Bsp_DMA_Stream_1))
        return NULL;

    if ((dma == Bsp_DMA_1) && ((stream < Bsp_DMA_Stream_Sum) && (stream >= Bsp_DMA_Stream_1)))
    {
        if (!dma1_clk_init)
        {
            crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
            dmamux_enable(DMA1, TRUE);
            dma1_clk_init = true;
        }

        return BspDMA1_Instance_List[stream];
    }
    else if ((dma == Bsp_DMA_2) && ((stream < Bsp_DMA_Stream_7) && (stream >= Bsp_DMA_Stream_1)))
    {
        if (!dma2_clk_init)
        {
            // crm_periph_clock_enable(CRM_DMA2_PERIPH_CLOCK, TRUE);
            dmamux_enable(DMA2, TRUE);
            dma2_clk_init = true;
        }

        return BspDMA2_Instance_List[stream];
    }

    return NULL;
}

static void BspDMA_EnableIRQ(BspDMA_List dma, BspDMA_Stream_List stream, uint32_t preempt, uint32_t sub)
{
    IRQn_Type irq;

    if ((dma < Bsp_DMA_1) || (stream < Bsp_DMA_Stream_1))
        return;

    if (dma == Bsp_DMA_1)
    {
        switch (stream)
        {
        case Bsp_DMA_Stream_1:
            irq = DMA1_Channel1_IRQn;
            break;

        case Bsp_DMA_Stream_2:
            irq = DMA1_Channel2_IRQn;
            break;

        case Bsp_DMA_Stream_3:
            irq = DMA1_Channel3_IRQn;
            break;

        case Bsp_DMA_Stream_4:
            irq = DMA1_Channel4_IRQn;
            break;

        case Bsp_DMA_Stream_5:
            irq = DMA1_Channel5_IRQn;
            break;

        case Bsp_DMA_Stream_6:
            irq = DMA1_Channel6_IRQn;
            break;

        case Bsp_DMA_Stream_7:
            irq = DMA1_Channel7_IRQn;
            break;

        default:
            return;
        }
    }
    else if (dma == Bsp_DMA_2)
    {
        switch (stream)
        {
        case Bsp_DMA_Stream_1:
            irq = DMA2_Channel1_IRQn;
            break;

        case Bsp_DMA_Stream_2:
            irq = DMA2_Channel2_IRQn;
            break;

        case Bsp_DMA_Stream_3:
            irq = DMA2_Channel3_IRQn;
            break;

        case Bsp_DMA_Stream_4:
            irq = DMA2_Channel4_IRQn;
            break;

        case Bsp_DMA_Stream_5:
            irq = DMA2_Channel5_IRQn;
            break;

        case Bsp_DMA_Stream_6:
            irq = DMA2_Channel6_IRQn;
            break;

        case Bsp_DMA_Stream_7:
            irq = DMA2_Channel7_IRQn;
            break;

        default:
            return;
        }
    }
    else
        return;

    nvic_priority_group_config(NVIC_PRIORITY_GROUP_0);
    nvic_irq_enable(irq, preempt, sub);
}

static bool BspDMA_Pipe_Init(BspDMA_Pipe_TransFin_Cb fin_cb, BspDMA_Pipe_TransErr_Cb err_cb)
{
    dma_init_type dma_init_struct;

    /* enable dma1 clock */
    crm_periph_clock_enable(CRM_DMA2_PERIPH_CLOCK, TRUE);

    /* dma1 channel1 configuration */
    dma_reset(DMA2_CHANNEL7);
    dma_init_struct.buffer_size = 0;
    dma_init_struct.direction = DMA_DIR_MEMORY_TO_MEMORY;
    dma_init_struct.memory_base_addr = (uint32_t)0;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_base_addr = (uint32_t)0;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
    dma_init_struct.peripheral_inc_enable = TRUE;
    dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
    dma_init_struct.loop_mode_enable = FALSE;
    dma_init(DMA2_CHANNEL7, &dma_init_struct);

    /* enable transfer full data intterrupt */
    dma_interrupt_enable(DMA2_CHANNEL7, DMA_FDT_INT, TRUE);

    /* dma1 channel1 interrupt nvic init */
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_0);
    nvic_irq_enable(DMA2_Channel7_IRQn, 5, 0);

    DataPipe_DMA_Init = true;

    DataPipe_Trans_Fin_Callback = fin_cb;
    DataPipe_Trans_Err_Callback = err_cb;

    return true;
}

static bool BspDMA_Pipe_Trans(uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
{
    if(!DataPipe_DMA_Init)
        return false;

    if((SrcAddress == 0) || (DstAddress == 0) || (DataLength == 0))
        return false;

    DMA2_CHANNEL7->maddr = DstAddress;
    DMA2_CHANNEL7->paddr = SrcAddress;
    DMA2_CHANNEL7->dtcnt_bit.cnt = DataLength;
    dma_channel_enable(DMA2_CHANNEL7, TRUE);

    return true;
}

static void *BspDMA_Get_Pipe_Handle(void)
{
    if(!DataPipe_DMA_Init)
        return NULL;

    return (void *)DMA2_CHANNEL7;
}

void BspDMA_Pipe_Irq_Callback(void)
{
    if(DataPipe_DMA_Init)
    {
        dma_channel_enable(DMA2_CHANNEL7, FALSE);

        if(dma_data_number_get(DMA2_CHANNEL7) != DMA2_CHANNEL7->dtcnt_bit.cnt)
        {
            /* piped data size error */
            if(DataPipe_Trans_Err_Callback)
                DataPipe_Trans_Err_Callback((void *)DMA2_CHANNEL7); 
        }
        else
        {
            if(DataPipe_Trans_Fin_Callback)
               DataPipe_Trans_Fin_Callback((void *)DMA2_CHANNEL7); 
        }
    }
}
