#include "DataPipe.h"
#include "Bsp_DMA.h"
#include "Srv_OsCommon.h"

#define MAX_RETRY_CNT 200
#define MAX_PIPE_FREQ 2000

/* internal variable */
static Data_PlugedPipeObj_TypeDef Cur_Pluged_PipeObj = {.org = NULL, .dst = NULL};
static DataPipe_State_List Pipe_State = Pipe_UnReady;

/* internal function */
static void DataPipe_TransFinish_Callback(void *dma_hdl);
static void DataPipe_TransError_Callback(void *dma_hdl);

bool DataPipe_Init(void)
{
    if(!BspDMA_Pipe.init(DataPipe_TransFinish_Callback, DataPipe_TransError_Callback))
        return false;

    Pipe_State = Pipe_Ready;

    return true;
}

bool DataPipe_DeInit(void)
{
    return BspDMA_Pipe.de_init();
}

bool DataPipe_SendTo(DataPipeObj_TypeDef *p_org, DataPipeObj_TypeDef *p_dst)
{
    uint8_t retry_cnt = 5;

    if ((p_org == NULL) ||
        (p_dst == NULL) ||
        (p_org->data_size != p_dst->data_size) ||
        !p_org->enable ||
        !p_dst->enable ||
        (p_dst->min_rx_interval &&
         p_dst->rx_ms_rt &&
         (SrvOsCommon.get_os_ms() - p_dst->rx_ms_rt < p_dst->min_rx_interval)))
        return false;

    retry_cnt = MAX_RETRY_CNT;
    while (Pipe_State != Pipe_Ready)
    {
        retry_cnt--;

        if (retry_cnt == 0)
            return false;
    }

    Cur_Pluged_PipeObj.dst = p_dst;
    Cur_Pluged_PipeObj.org = p_org;

retry:
    Pipe_State = Pipe_Busy;
    if (!BspDMA_Pipe.trans(p_org->data_addr, p_dst->data_addr, p_org->data_size))
    {
        retry_cnt--;

        if (retry_cnt)
            goto retry;

        Pipe_State = Pipe_Error;
        p_org->er_cnt++;
        p_dst->er_cnt++;
    }

    return true;
}

/* still in developing */
bool DataPipe_DealError(void)
{
    if (Pipe_State == Pipe_Error)
    {
    }

    return true;
}

/* transmit completely callback */
static void DataPipe_TransFinish_Callback(void *dma_hdl)
{
    uint32_t cur_ms = SrvOsCommon.get_os_ms();

    if (BspDMA_Pipe.get_hanle && (To_DMA_Handle_Ptr(dma_hdl) == To_DMA_Handle_Ptr(BspDMA_Pipe.get_hanle())))
    {
        Pipe_State = Pipe_Ready;

        Cur_Pluged_PipeObj.dst->rx_cnt++;
        Cur_Pluged_PipeObj.org->tx_cnt++;

        if (Cur_Pluged_PipeObj.org->trans_finish_cb)
            Cur_Pluged_PipeObj.org->trans_finish_cb(Cur_Pluged_PipeObj.org);

        if (Cur_Pluged_PipeObj.dst->trans_finish_cb)
            Cur_Pluged_PipeObj.dst->trans_finish_cb(Cur_Pluged_PipeObj.dst);

        if (Cur_Pluged_PipeObj.dst->rx_ms_rt)
            Cur_Pluged_PipeObj.dst->detect_interval = cur_ms - Cur_Pluged_PipeObj.dst->rx_ms_rt;

        Cur_Pluged_PipeObj.dst->rx_ms_rt = cur_ms;

        Cur_Pluged_PipeObj.dst = NULL;
        Cur_Pluged_PipeObj.org = NULL;
    }
}

/* transmit error callback */
static void DataPipe_TransError_Callback(void *dma_hdl)
{
    if (BspDMA_Pipe.get_hanle && (To_DMA_Handle_Ptr(dma_hdl) == To_DMA_Handle_Ptr(BspDMA_Pipe.get_hanle())))
    {
        Pipe_State = Pipe_Error;

        Cur_Pluged_PipeObj.dst->er_cnt++;
        Cur_Pluged_PipeObj.org->er_cnt++;

        if (Cur_Pluged_PipeObj.org->trans_error_cb)
            Cur_Pluged_PipeObj.org->trans_error_cb(Cur_Pluged_PipeObj.org);

        if (Cur_Pluged_PipeObj.dst->trans_error_cb)
            Cur_Pluged_PipeObj.dst->trans_error_cb(Cur_Pluged_PipeObj.dst);

        Cur_Pluged_PipeObj.dst = NULL;
        Cur_Pluged_PipeObj.org = NULL;

#if defined STM32H743xx
        /*!< Transfer error */
        if (To_DMA_Handle_Ptr(dma_hdl)->ErrorCode & HAL_DMA_ERROR_TE)
        {
        }

        /*!< FIFO error */
        if (To_DMA_Handle_Ptr(dma_hdl)->ErrorCode & HAL_DMA_ERROR_FE)
        {
        }

        /*!< Direct Mode error */
        if (To_DMA_Handle_Ptr(dma_hdl)->ErrorCode & HAL_DMA_ERROR_DME)
        {
        }

        /*!< Timeout error */
        if (To_DMA_Handle_Ptr(dma_hdl)->ErrorCode & HAL_DMA_ERROR_TIMEOUT)
        {
        }

        /*!< Parameter error */
        if (To_DMA_Handle_Ptr(dma_hdl)->ErrorCode & HAL_DMA_ERROR_PARAM)
        {
        }

        /*!< Abort requested with no Xfer ongoing */
        if (To_DMA_Handle_Ptr(dma_hdl)->ErrorCode & HAL_DMA_ERROR_PARAM)
        {
        }

        /*!< Not supported mode */
        if (To_DMA_Handle_Ptr(dma_hdl)->ErrorCode & HAL_DMA_ERROR_NOT_SUPPORTED)
        {
        }

        /*!< DMAMUX sync overrun  error */
        if (To_DMA_Handle_Ptr(dma_hdl)->ErrorCode & HAL_DMA_ERROR_SYNC)
        {
        }

        /*!< DMAMUX request generator overrun  error */
        if (To_DMA_Handle_Ptr(dma_hdl)->ErrorCode & HAL_DMA_ERROR_REQGEN)
        {
        }

        /*!< DMA Busy error */
        if (To_DMA_Handle_Ptr(dma_hdl)->ErrorCode & HAL_DMA_ERROR_BUSY)
        {
        }
#endif
        /* recover from error */
    }
}
