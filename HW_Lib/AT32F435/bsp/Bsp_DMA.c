#include "Bsp_DMA.h"
#include "at32f435_437_dma.h"

// static DMA_HandleTypeDef *BspDMA_Map[Bsp_DMA_Sum][Bsp_DMA_Stream_Sum] = {NULL};

static const BspDMA1_Instance_List[Bsp_DMA_Stream_Sum] = {
    DMA1_CHANNEL1,
    DMA1_CHANNEL2,
    DMA1_CHANNEL3,
    DMA1_CHANNEL4,
    DMA1_CHANNEL5,
    DMA1_CHANNEL6,
    DMA1_CHANNEL7};

static const BspDMA2_Instance_List[Bsp_DMA_Stream_Sum] = {
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

/* external function */
static bool BspDMA_Pipe_Init(BspDMA_Pipe_TransFin_Cb fin_cb, BspDMA_Pipe_TransErr_Cb err_cb);
static bool BspDMA_Pipe_Trans(uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
static void *BspDMA_Get_Pipe_Handle(void);

BspDMA_Pipe_TypeDef BspDMA_Pipe = {
    .init = BspDMA_Pipe_Init,
    .trans = BspDMA_Pipe_Trans,
    .get_hanle = BspDMA_Get_Pipe_Handle,
};

static dma_channel_type *BspDMA_Get_Instance(BspDMA_List dma, BspDMA_Stream_List stream)
{
    if ((dma < Bsp_DMA_1) || (stream < Bsp_DMA_Stream_1))
        return NULL;

    if ((dma == Bsp_DMA_1) && ((stream < Bsp_DMA_Stream_Sum) && (stream >= Bsp_DMA_Stream_1)))
    {

    }
    else if ((dma == Bsp_DMA_2) && ((stream < Bsp_DMA_Stream_7) && (stream >= Bsp_DMA_Stream_1)))
    {

    }

    return NULL;
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
