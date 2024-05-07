#include "Bsp_DMA.h"
#include "at32f435_437_dma.h"

static BspDMA_IrqCall_Obj_TypeDef* BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_Sum] = {NULL};
static BspDMA_IrqCall_Obj_TypeDef* BspDMA2_Irq_Callback_List[Bsp_DMA_Stream_Sum] = {NULL};

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
static bool Dma_Clock_Init = false;
static BspDMA_Pipe_TransFin_Cb DataPipe_Trans_Fin_Callback = NULL;
static BspDMA_Pipe_TransErr_Cb DataPipe_Trans_Err_Callback = NULL;

/* external funtion */
static bool BspDMA_Regist_Obj(BspDMA_List dma, BspDMA_Stream_List stream, void *hdl);
static bool BspDMA_Unregist_Obj(BspDMA_List dma, BspDMA_Stream_List stream);
static void *BspDMA_Get_Handle(BspDMA_List dma, BspDMA_Stream_List stream);
static dma_channel_type *BspDMA_Get_Instance(BspDMA_List dma, BspDMA_Stream_List stream);
static void BspDMA_EnableIRQ(BspDMA_List dma, BspDMA_Stream_List stream, uint32_t preempt, uint32_t sub, uint32_t mux_seq, void *cb);
static void *BspDMA_Get_Channel_Instance(int8_t dma, int8_t stream);

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
    .get_channel_instance = BspDMA_Get_Channel_Instance,
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

static void* BspDMA_Get_Channel_Instance(int8_t dma, int8_t stream)
{
    if ((dma == Bsp_DMA_None) && \
        (dma == Bsp_DMA_Sum) && \
        (stream == Bsp_DMA_Stream_None) && \
        (stream == Bsp_DMA_Stream_Sum))
        return 0;
    
    if (dma == Bsp_DMA_1)
    {
        switch (stream)
        {
            case Bsp_DMA_Stream_1:
                return DMA1_CHANNEL1;

            case Bsp_DMA_Stream_2:
                return DMA1_CHANNEL2;
            
            case Bsp_DMA_Stream_3:
                return DMA1_CHANNEL3;

            case Bsp_DMA_Stream_4:
                return DMA1_CHANNEL4;

            case Bsp_DMA_Stream_5:
                return DMA1_CHANNEL5;

            case Bsp_DMA_Stream_6:
                return DMA1_CHANNEL6;

            case Bsp_DMA_Stream_7:
                return DMA1_CHANNEL7;

            default:
                return 0;
        }
    }
    else if (dma == Bsp_DMA_2)
    {
        switch (stream)
        {
            case Bsp_DMA_Stream_1:
                return DMA2_CHANNEL1;

            case Bsp_DMA_Stream_2:
                return DMA2_CHANNEL2;
            
            case Bsp_DMA_Stream_3:
                return DMA2_CHANNEL3;

            case Bsp_DMA_Stream_4:
                return DMA2_CHANNEL4;

            case Bsp_DMA_Stream_5:
                return DMA2_CHANNEL5;

            case Bsp_DMA_Stream_6:
                return DMA2_CHANNEL6;

            case Bsp_DMA_Stream_7:
            default:
                return 0;
        }
    }

    return 0;
}

static void *BspDMA_Get_Handle(BspDMA_List dma, BspDMA_Stream_List stream)
{
    UNUSED(dma);
    UNUSED(stream);

    return NULL;
}

static dma_channel_type *BspDMA_Get_Instance(BspDMA_List dma, BspDMA_Stream_List stream)
{
    static bool dma1_enable = false;
    static bool dma2_enable = false;

    if ((dma < Bsp_DMA_1) || (stream < Bsp_DMA_Stream_1))
        return NULL;

    /* enable dma1 clock */
    if (!Dma_Clock_Init)
    {
        crm_periph_clock_enable(CRM_DMA2_PERIPH_CLOCK, TRUE);
        crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
        Dma_Clock_Init = true;
    }

    if ((dma == Bsp_DMA_1) && ((stream < Bsp_DMA_Stream_Sum) && (stream >= Bsp_DMA_Stream_1)))
    {
        if (!dma1_enable)
        {
            // crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
            dmamux_enable(DMA1, TRUE);
            dma1_enable = true;
        }

        return (dma_channel_type *)BspDMA1_Instance_List[stream];
    }
    else if ((dma == Bsp_DMA_2) && ((stream < Bsp_DMA_Stream_7) && (stream >= Bsp_DMA_Stream_1)))
    {
        if (!dma2_enable)
        {
            // crm_periph_clock_enable(CRM_DMA2_PERIPH_CLOCK, TRUE);
            dmamux_enable(DMA2, TRUE);
            dma2_enable = true;
        }

        return (dma_channel_type *)BspDMA2_Instance_List[stream];
    }

    return NULL;
}

static void BspDMA_EnableIRQ(BspDMA_List dma, BspDMA_Stream_List stream, uint32_t preempt, uint32_t sub, uint32_t mux_seq, void *cb)
{
    IRQn_Type irq;
    dmamux_channel_type *dmamux = NULL;

    if ((dma < Bsp_DMA_1) || (stream < Bsp_DMA_Stream_1))
        return;

    if (dma == Bsp_DMA_1)
    {
        switch (stream)
        {
        case Bsp_DMA_Stream_1:
            irq = DMA1_Channel1_IRQn;
            dmamux = DMA1MUX_CHANNEL1;
            break;

        case Bsp_DMA_Stream_2:
            irq = DMA1_Channel2_IRQn;
            dmamux = DMA1MUX_CHANNEL2;
            break;

        case Bsp_DMA_Stream_3:
            irq = DMA1_Channel3_IRQn;
            dmamux = DMA1MUX_CHANNEL3;
            break;

        case Bsp_DMA_Stream_4:
            irq = DMA1_Channel4_IRQn;
            dmamux = DMA1MUX_CHANNEL4;
            break;

        case Bsp_DMA_Stream_5:
            irq = DMA1_Channel5_IRQn;
            dmamux = DMA1MUX_CHANNEL5;
            break;

        case Bsp_DMA_Stream_6:
            irq = DMA1_Channel6_IRQn;
            dmamux = DMA1MUX_CHANNEL6;
            break;

        case Bsp_DMA_Stream_7:
            irq = DMA1_Channel7_IRQn;
            dmamux = DMA1MUX_CHANNEL7;
            break;

        default:
            return;
        }

        BspDMA1_Irq_Callback_List[stream] = (BspDMA_IrqCall_Obj_TypeDef *)cb;
    }
    else if (dma == Bsp_DMA_2)
    {
        switch (stream)
        {
        case Bsp_DMA_Stream_1:
            irq = DMA2_Channel1_IRQn;
            dmamux = DMA2MUX_CHANNEL1;
            break;

        case Bsp_DMA_Stream_2:
            irq = DMA2_Channel2_IRQn;
            dmamux = DMA2MUX_CHANNEL2;
            break;

        case Bsp_DMA_Stream_3:
            irq = DMA2_Channel3_IRQn;
            dmamux = DMA2MUX_CHANNEL3;
            break;

        case Bsp_DMA_Stream_4:
            irq = DMA2_Channel4_IRQn;
            dmamux = DMA2MUX_CHANNEL4;
            break;

        case Bsp_DMA_Stream_5:
            irq = DMA2_Channel5_IRQn;
            dmamux = DMA2MUX_CHANNEL5;
            break;

        case Bsp_DMA_Stream_6:
            irq = DMA2_Channel6_IRQn;
            dmamux = DMA2MUX_CHANNEL6;
            break;

        case Bsp_DMA_Stream_7:
            irq = DMA2_Channel7_IRQn;
            dmamux = DMA2MUX_CHANNEL7;
            break;

        default:
            return;
        }
    
        BspDMA2_Irq_Callback_List[stream] = (BspDMA_IrqCall_Obj_TypeDef *)cb;
    }
    else
        return;

    if((mux_seq < DMAMUX_DMAREQ_ID_DVP) || (mux_seq > DMAMUX_DMAREQ_ID_REQ_G1))
        dmamux_init(dmamux, (dmamux_requst_id_sel_type)mux_seq);

    dma_interrupt_enable(BspDMA_Get_Instance(dma, stream), DMA_FDT_INT, TRUE);
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    nvic_irq_enable(irq, preempt, sub);
}

static bool BspDMA_Pipe_Init(BspDMA_Pipe_TransFin_Cb fin_cb, BspDMA_Pipe_TransErr_Cb err_cb)
{
    dma_init_type dma_init_struct;

    /* enable dma1 clock */
    if (!Dma_Clock_Init)
    {
        crm_periph_clock_enable(CRM_DMA2_PERIPH_CLOCK, TRUE);
        crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
        Dma_Clock_Init = true;
    }

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
    dma_init_struct.priority = DMA_PRIORITY_VERY_HIGH;
    dma_init_struct.loop_mode_enable = FALSE;
    dma_init(DMA2_CHANNEL7, &dma_init_struct);

    /* enable transfer full data intterrupt */
    dma_interrupt_enable(DMA2_CHANNEL7, DMA_FDT_INT, TRUE);

    /* dma1 channel1 interrupt nvic init */
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
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

void BspDMA_Irq_Callback(void *channel)
{
    dma_channel_type *ch_tmp = To_DMA_Handle_Ptr(channel);
    void *cus_data = NULL;

    if (ch_tmp == DMA1_CHANNEL1)
    {
        if (BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_1] && \
            BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_1]->BspDMA_Irq_Callback_Func)
        {
            cus_data = BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_1]->cus_data;
            BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_1]->BspDMA_Irq_Callback_Func(cus_data);
        }

        return;
    }

    if (ch_tmp == DMA1_CHANNEL2)
    {
        if (BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_2] && \
            BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_2]->BspDMA_Irq_Callback_Func)
        {
            cus_data = BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_2]->cus_data;
            BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_2]->BspDMA_Irq_Callback_Func(cus_data);
        }

        return;
    }

    if (ch_tmp == DMA1_CHANNEL3)
    {
        if (BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_3] && \
            BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_3]->BspDMA_Irq_Callback_Func)
        {
            cus_data = BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_3]->cus_data;
            BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_3]->BspDMA_Irq_Callback_Func(cus_data);
        }

        return;
    }

    if (ch_tmp == DMA1_CHANNEL4)
    {
        if (BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_4] && \
            BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_4]->BspDMA_Irq_Callback_Func)
        {
            cus_data = BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_4]->cus_data;
            BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_4]->BspDMA_Irq_Callback_Func(cus_data);
        }

        return;
    }

    if (ch_tmp == DMA1_CHANNEL5)
    {
        if (BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_5] && \
            BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_5]->BspDMA_Irq_Callback_Func)
        {
            cus_data = BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_5]->cus_data;
            BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_5]->BspDMA_Irq_Callback_Func(cus_data);
        }

        return;
    }

    if (ch_tmp == DMA1_CHANNEL6)
    {
        if (BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_6] && \
            BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_6]->BspDMA_Irq_Callback_Func)
        {
            cus_data = BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_6]->cus_data;
            BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_6]->BspDMA_Irq_Callback_Func(cus_data);
        }

        return;
    }

    if (ch_tmp == DMA1_CHANNEL7)
    {
        if (BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_7] && \
            BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_7]->BspDMA_Irq_Callback_Func)
        {
            cus_data = BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_7]->cus_data;
            BspDMA1_Irq_Callback_List[Bsp_DMA_Stream_7]->BspDMA_Irq_Callback_Func(cus_data);
        }

        return;
    }

    if (ch_tmp == DMA2_CHANNEL1)
    {
        if (BspDMA2_Irq_Callback_List[Bsp_DMA_Stream_1] && \
            BspDMA2_Irq_Callback_List[Bsp_DMA_Stream_1]->BspDMA_Irq_Callback_Func)
        {
            cus_data = BspDMA2_Irq_Callback_List[Bsp_DMA_Stream_1]->cus_data;
            BspDMA2_Irq_Callback_List[Bsp_DMA_Stream_1]->BspDMA_Irq_Callback_Func(cus_data);
        }
    
        return;
    }

    if (ch_tmp == DMA2_CHANNEL2)
    {
        if (BspDMA2_Irq_Callback_List[Bsp_DMA_Stream_2] && \
            BspDMA2_Irq_Callback_List[Bsp_DMA_Stream_2]->BspDMA_Irq_Callback_Func)
        {
            cus_data = BspDMA2_Irq_Callback_List[Bsp_DMA_Stream_2]->cus_data;
            BspDMA2_Irq_Callback_List[Bsp_DMA_Stream_2]->BspDMA_Irq_Callback_Func(cus_data);
        }
    
        return;
    }

    if (ch_tmp == DMA2_CHANNEL3)
    {
        if (BspDMA2_Irq_Callback_List[Bsp_DMA_Stream_3] && \
            BspDMA2_Irq_Callback_List[Bsp_DMA_Stream_3]->BspDMA_Irq_Callback_Func)
        {
            cus_data = BspDMA2_Irq_Callback_List[Bsp_DMA_Stream_3]->cus_data;
            BspDMA2_Irq_Callback_List[Bsp_DMA_Stream_3]->BspDMA_Irq_Callback_Func(cus_data);
        }

        return;
    }

    if (ch_tmp == DMA2_CHANNEL4)
    {
        if (BspDMA2_Irq_Callback_List[Bsp_DMA_Stream_4] && \
            BspDMA2_Irq_Callback_List[Bsp_DMA_Stream_4]->BspDMA_Irq_Callback_Func)
        {
            cus_data = BspDMA2_Irq_Callback_List[Bsp_DMA_Stream_4]->cus_data;
            BspDMA2_Irq_Callback_List[Bsp_DMA_Stream_4]->BspDMA_Irq_Callback_Func(cus_data);
        }

        return;
    }

    if (ch_tmp == DMA2_CHANNEL5)
    {
        if (BspDMA2_Irq_Callback_List[Bsp_DMA_Stream_5] && \
            BspDMA2_Irq_Callback_List[Bsp_DMA_Stream_5]->BspDMA_Irq_Callback_Func)
        {
            cus_data = BspDMA2_Irq_Callback_List[Bsp_DMA_Stream_5]->cus_data;
            BspDMA2_Irq_Callback_List[Bsp_DMA_Stream_5]->BspDMA_Irq_Callback_Func(cus_data);
        }

        return;
    }

    if (ch_tmp == DMA2_CHANNEL6)
    {
        if (BspDMA2_Irq_Callback_List[Bsp_DMA_Stream_6] && \
            BspDMA2_Irq_Callback_List[Bsp_DMA_Stream_6]->BspDMA_Irq_Callback_Func)
        {
            cus_data = BspDMA2_Irq_Callback_List[Bsp_DMA_Stream_6]->cus_data;
            BspDMA2_Irq_Callback_List[Bsp_DMA_Stream_6]->BspDMA_Irq_Callback_Func(cus_data);
        }

        return;
    }
}
