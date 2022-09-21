#include "DataPool.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal_dma.h"

DMA_HandleTypeDef hdma_memtomem_dma2_stream7;

static bool Pool_Init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* Configure DMA request hdma_memtomem_dma2_stream7 on DMA2_Stream7 */
    hdma_memtomem_dma2_stream7.Instance = DMA2_Stream7;
    hdma_memtomem_dma2_stream7.Init.Request = DMA_REQUEST_MEM2MEM;
    hdma_memtomem_dma2_stream7.Init.Direction = DMA_MEMORY_TO_MEMORY;
    hdma_memtomem_dma2_stream7.Init.PeriphInc = DMA_PINC_ENABLE;
    hdma_memtomem_dma2_stream7.Init.MemInc = DMA_MINC_ENABLE;
    hdma_memtomem_dma2_stream7.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_memtomem_dma2_stream7.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_memtomem_dma2_stream7.Init.Mode = DMA_NORMAL;
    hdma_memtomem_dma2_stream7.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_memtomem_dma2_stream7.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_memtomem_dma2_stream7.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_memtomem_dma2_stream7.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_memtomem_dma2_stream7.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_memtomem_dma2_stream7) != HAL_OK)
        return false;

    /* DMA interrupt init */
    /* DMA2_Stream7_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

    return true;
}


// HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma);
// HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *hdma);
// HAL_StatusTypeDef HAL_DMA_Start (DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
// HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
// HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma);
// HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma);
// HAL_StatusTypeDef HAL_DMA_PollForTransfer(DMA_HandleTypeDef *hdma, HAL_DMA_LevelCompleteTypeDef CompleteLevel, uint32_t Timeout);
// void              HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);
// HAL_StatusTypeDef HAL_DMA_RegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID, void (* pCallback)(DMA_HandleTypeDef *_hdma));
// HAL_StatusTypeDef HAL_DMA_UnRegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID);
// HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma);
// uint32_t             HAL_DMA_GetError(DMA_HandleTypeDef *hdma);