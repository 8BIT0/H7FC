#include "DataPipe.h"
#include "mmu.h"
#include "kernel.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal_dma.h"
#include "queue.h"

DMA_HandleTypeDef DataPipe_DMA;

/* internal function */
static void DataPipe_TransFinish_Callback(DMA_HandleTypeDef *dma_hdl);
static void DataPipe_TransError_Callback(DMA_HandleTypeDef *dma_hdl);

static bool DataPipe_Init(void)
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
    DataPipe_DMA.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    DataPipe_DMA.Init.MemBurst = DMA_MBURST_SINGLE;
    DataPipe_DMA.Init.PeriphBurst = DMA_PBURST_SINGLE;

    /* set transmit process callback */
    HAL_DMA_RegisterCallback(&DataPipe_DMA, HAL_DMA_XFER_CPLT_CB_ID, DataPipe_TransFinish_Callback);
    HAL_DMA_RegisterCallback(&DataPipe_DMA, HAL_DMA_XFER_ERROR_CB_ID, DataPipe_TransError_Callback);

    if (HAL_DMA_Init(&DataPipe_DMA) != HAL_OK)
        return false;

    /* DMA interrupt init */
    /* DMA2_Stream7_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

    return true;
}

static DataPipe_Handle DataPipe_Create()
{
}

static bool DataPipe_SendTo()
{
    // if ((p_org == NULL) || (p_dst == NULL) || (size == 0))
    //     return false;

    Kernel_EnterCritical();

    Kernel_ExitCritical();

    return true;
}

/* transmit completely callback */
static void DataPipe_TransFinish_Callback(DMA_HandleTypeDef *dma_hdl)
{
    if (dma_hdl == &DataPipe_DMA)
    {
    }
}

/* transmit error callback */
static void DataPipe_TransError_Callback(DMA_HandleTypeDef *dma_hdl)
{
    if (dma_hdl == &DataPipe_DMA)
    {
    }
}

// HAL_StatusTypeDef HAL_DMA_Start (DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
// HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
// HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma);
// HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma);
// HAL_StatusTypeDef HAL_DMA_PollForTransfer(DMA_HandleTypeDef *hdma, HAL_DMA_LevelCompleteTypeDef CompleteLevel, uint32_t Timeout);
// void              HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);
// HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma);
// uint32_t             HAL_DMA_GetError(DMA_HandleTypeDef *hdma);