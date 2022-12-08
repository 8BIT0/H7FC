/*
 * coder : 8_B!T0
 * bref : default using uart dma transfer
 */
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_uart.h"
#include "stm32h7xx_hal_dma.h"
#include "Bsp_Uart.h"

static bool BspUart_Init_Clock(BspUARTObj_TypeDef *obj)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    DMA_HandleTypeDef dma_cfg;
    IRQn_Type irqn;

    dma_cfg.Init.PeriphInc = DMA_PINC_DISABLE;
    dma_cfg.Init.MemInc = DMA_MINC_ENABLE;
    dma_cfg.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    dma_cfg.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    dma_cfg.Init.Mode = DMA_NORMAL;
    dma_cfg.Init.Priority = DMA_PRIORITY_MEDIUM;
    dma_cfg.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    if( (obj == NULL) || 
        (obj->hdl.Instance == NULL) || 
        (obj->tx_dma_instance == NULL) ||
        (obj->rx_dma_instance == NULL))
        return false;

    if(obj->hdl.Instance == UART4)
    {
        __HAL_RCC_UART4_CLK_ENABLE();

        /* pin clock and pin config */

        dma_cfg.Instance = obj->rx_dma_instance;
        dma_cfg.Init.Request = DMA_REQUEST_UART4_RX;
        dma_cfg.Init.Direction = DMA_PERIPH_TO_MEMORY;
        
        if (HAL_DMA_Init(&dma_cfg) != HAL_OK)
            return false;

        __HAL_LINKDMA(huart, hdmarx, dma_cfg);

        dma_cfg.Instance = obj->tx_dma_instance;
        dma_cfg.Init.Request = DMA_REQUEST_UART4_TX;
        dma_cfg.Init.Direction = DMA_MEMORY_TO_PERIPH;

        if (HAL_DMA_Init(&dma_cfg) != HAL_OK)
            return false;

        __HAL_LINKDMA(huart, hdmatx, dma_cfg);

        irqn = UART4_IRQn;
    }

    if(obj->hdl.Instance == UART6)
    {
        __HAL_RCC_USART6_CLK_ENABLE();

        /* pin clock and pin config */

        irqn = UART6_IRQn;
    }

    if(obj->hdl.Instance == UART7)
    {
        __HAL_RCC_UART7_CLK_ENABLE();

        /* pin clock and pin config */

        irqn = UART7_IRQn;
    }

    HAL_NVIC_SetPriority(irqn, 0, 0);
    HAL_NVIC_EnableIRQ(irqn);

    return true;
}

static bool BspUart_Init(BspUARTObj_TypeDef *obj)
{
    UART_HandleTypeDef uart_cfg;

    uart_cfg.Instance = obj->hdl.Instance;
    uart_cfg.Init.BaudRate = obj->baudrate;
    uart_cfg.Init.WordLength = UART_WORDLENGTH_8B;
    uart_cfg.Init.StopBits = UART_STOPBITS_1;
    uart_cfg.Init.Parity = UART_PARITY_NONE;
    uart_cfg.Init.Mode = UART_MODE_TX_RX;
    uart_cfg.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart_cfg.Init.OverSampling = UART_OVERSAMPLING_16;
    uart_cfg.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    uart_cfg.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    uart_cfg.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT | UART_ADVFEATURE_DMADISABLEONERROR_INIT;
    uart_cfg.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
    uart_cfg.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;

    /* swap tx rx pin */
    if(obj->pin_swap)
        uart_cfg.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;

    if (HAL_UART_Init(&uart_cfg) != HAL_OK ||
        HAL_UARTEx_SetTxFifoThreshold(&uart_cfg, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK ||
        HAL_UARTEx_SetRxFifoThreshold(&uart_cfg, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK ||
        HAL_UARTEx_DisableFifoMode(&uart_cfg) != HAL_OK)
        return false;

    return true;
}
