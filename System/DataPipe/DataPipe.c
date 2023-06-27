#include "DataPipe.h"
#include "kernel.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_dma.h"
#include "../../DataStructure/queue.h"
#include "Srv_OsCommon.h"

#define MAX_RETRY_CNT 200
#define MAX_PIPE_FREQ 2000

DMA_HandleTypeDef DataPipe_DMA;

/* internal variable */
static Data_PlugedPipeObj_TypeDef Cur_Pluged_PipeObj = {.org = NULL, .dst = NULL};
static DataPipe_State_List Pipe_State = Pipe_UnReady;

/* internal function */
static void DataPipe_TransFinish_Callback(DMA_HandleTypeDef *dma_hdl);
static void DataPipe_TransError_Callback(DMA_HandleTypeDef *dma_hdl);

bool DataPipe_Init(void)
{
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
    HAL_DMA_RegisterCallback(&DataPipe_DMA, HAL_DMA_XFER_CPLT_CB_ID, DataPipe_TransFinish_Callback);
    HAL_DMA_RegisterCallback(&DataPipe_DMA, HAL_DMA_XFER_ERROR_CB_ID, DataPipe_TransError_Callback);

    /* DMA interrupt init */
    /* DMA2_Stream7_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

    Pipe_State = Pipe_Ready;

    return true;
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
         p_dst->rx_us_rt &&
         (Get_CurrentRunningUs() - p_dst->rx_us_rt < p_dst->min_rx_interval)))
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

    Kernel_EnterCritical();

retry:
    Pipe_State = Pipe_Busy;
    if (HAL_DMA_Start_IT(&DataPipe_DMA, p_org->data_addr, p_dst->data_addr, p_org->data_size) != HAL_OK)
    {
        retry_cnt--;

        if (HAL_DMA_Abort_IT(&DataPipe_DMA) != HAL_OK)
        {
            if (retry_cnt)
                goto retry;
        }

        Pipe_State = Pipe_Error;
        p_org->er_cnt++;
        p_dst->er_cnt++;
    }

    Kernel_ExitCritical();

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
static void DataPipe_TransFinish_Callback(DMA_HandleTypeDef *dma_hdl)
{
    uint64_t cur_us = Get_CurrentRunningUs();

    if (dma_hdl == &DataPipe_DMA)
    {
        Pipe_State = Pipe_Ready;

        Cur_Pluged_PipeObj.dst->rx_cnt++;
        Cur_Pluged_PipeObj.org->tx_cnt++;

        if (Cur_Pluged_PipeObj.org->trans_finish_cb)
            Cur_Pluged_PipeObj.org->trans_finish_cb(Cur_Pluged_PipeObj.org);

        if (Cur_Pluged_PipeObj.dst->trans_finish_cb)
            Cur_Pluged_PipeObj.dst->trans_finish_cb(Cur_Pluged_PipeObj.dst);

        if (Cur_Pluged_PipeObj.dst->rx_us_rt)
            Cur_Pluged_PipeObj.dst->detect_interval = cur_us - Cur_Pluged_PipeObj.dst->rx_us_rt;

        Cur_Pluged_PipeObj.dst->rx_us_rt = cur_us;

        Cur_Pluged_PipeObj.dst = NULL;
        Cur_Pluged_PipeObj.org = NULL;
    }
}

/* transmit error callback */
static void DataPipe_TransError_Callback(DMA_HandleTypeDef *dma_hdl)
{
    if (dma_hdl == &DataPipe_DMA)
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

        /*!< Transfer error */
        if (dma_hdl->ErrorCode & HAL_DMA_ERROR_TE)
        {
        }

        /*!< FIFO error */
        if (dma_hdl->ErrorCode & HAL_DMA_ERROR_FE)
        {
        }

        /*!< Direct Mode error */
        if (dma_hdl->ErrorCode & HAL_DMA_ERROR_DME)
        {
        }

        /*!< Timeout error */
        if (dma_hdl->ErrorCode & HAL_DMA_ERROR_TIMEOUT)
        {
        }

        /*!< Parameter error */
        if (dma_hdl->ErrorCode & HAL_DMA_ERROR_PARAM)
        {
        }

        /*!< Abort requested with no Xfer ongoing */
        if (dma_hdl->ErrorCode & HAL_DMA_ERROR_PARAM)
        {
        }

        /*!< Not supported mode */
        if (dma_hdl->ErrorCode & HAL_DMA_ERROR_NOT_SUPPORTED)
        {
        }

        /*!< DMAMUX sync overrun  error */
        if (dma_hdl->ErrorCode & HAL_DMA_ERROR_SYNC)
        {
        }

        /*!< DMAMUX request generator overrun  error */
        if (dma_hdl->ErrorCode & HAL_DMA_ERROR_REQGEN)
        {
        }

        /*!< DMA Busy error */
        if (dma_hdl->ErrorCode & HAL_DMA_ERROR_BUSY)
        {
        }

        /* recover from error */
    }
}
