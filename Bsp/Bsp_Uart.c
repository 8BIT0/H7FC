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
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    DMA_HandleTypeDef tx_dma_cfg;
    DMA_HandleTypeDef rx_dma_cfg;
    IRQn_Type irqn;

    tx_dma_cfg.Init.PeriphInc = DMA_PINC_DISABLE;
    tx_dma_cfg.Init.MemInc = DMA_MINC_ENABLE;
    tx_dma_cfg.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    tx_dma_cfg.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    tx_dma_cfg.Init.Mode = DMA_NORMAL;
    tx_dma_cfg.Init.Priority = DMA_PRIORITY_MEDIUM;
    tx_dma_cfg.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    tx_dma_cfg.Init.Direction = DMA_MEMORY_TO_PERIPH;

    rx_dma_cfg.Init.Direction = DMA_PERIPH_TO_MEMORY;
    rx_dma_cfg.Init.PeriphInc = DMA_PINC_DISABLE;
    rx_dma_cfg.Init.MemInc = DMA_MINC_ENABLE;
    rx_dma_cfg.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    rx_dma_cfg.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    rx_dma_cfg.Init.Mode = DMA_NORMAL;
    rx_dma_cfg.Init.Priority = DMA_PRIORITY_MEDIUM;
    rx_dma_cfg.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if( (obj == NULL) || 
        (obj->hdl.Instance == NULL) || 
        (obj->tx_dma_instance == NULL) ||
        (obj->rx_dma_instance == NULL))
        return false;

    if(obj->hdl.Instance == UART4)
    {
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART4;
        PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;

        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
            return false;

        __HAL_RCC_UART4_CLK_ENABLE();

        /* config */
        obj->tx_io.alternate = GPIO_AF8_UART4;
        obj->rx_io.alternate = GPIO_AF8_UART4;

        rx_dma_cfg.Instance = obj->rx_dma_instance;
        rx_dma_cfg.Init.Request = DMA_REQUEST_UART4_RX;

        tx_dma_cfg.Instance = obj->tx_dma_instance;
        tx_dma_cfg.Init.Request = DMA_REQUEST_UART4_TX;

        irqn = UART4_IRQn;
    }
    else if(obj->hdl.Instance == UART6)
    {
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART6;
        PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
        
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
            return false;

        __HAL_RCC_USART6_CLK_ENABLE();

        /* config */
        obj->tx_io.alternate = GPIO_AF7_USART6;
        obj->rx_io.alternate = GPIO_AF7_USART6;

        rx_dma_cfg.Instance = obj->rx_dma_instance;
        rx_dma_cfg.Init.Request = DMA_REQUEST_USART6_RX;

        tx_dma_cfg.Instance = obj->tx_dma_instance;
        tx_dma_cfg.Init.Request = DMA_REQUEST_USART6_TX;

        irqn = USART6_IRQn;
    }
    else if(obj->hdl.Instance == UART7)
    {
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART7;
        PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;

        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
            return false;

        __HAL_RCC_UART7_CLK_ENABLE();

        /* config */
        obj->tx_io.alternate = GPIO_AF7_UART7;
        obj->rx_io.alternate = GPIO_AF7_UART7;

        rx_dma_cfg.Instance = obj->rx_dma_instance;
        rx_dma_cfg.Init.Request = DMA_REQUEST_UART7_RX;

        tx_dma_cfg.Instance = obj->tx_dma_instance;
        tx_dma_cfg.Init.Request = DMA_REQUEST_UART7_TX;

        irqn = UART7_IRQn;
    }
    else
        return false;

    /* rx pin init */
    BspGPIO.alt_init(obj->rx_io);
    
    /* tx pin init */
    BspGPIO.alt_init(obj->tx_io);

    if (HAL_DMA_Init(&rx_dma_cfg) != HAL_OK)
        return false;

    __HAL_LINKDMA(huart, hdmarx, rx_dma_cfg);

    if (HAL_DMA_Init(&tx_dma_cfg) != HAL_OK)
        return false;

    __HAL_LINKDMA(huart, hdmatx, tx_dma_cfg);

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
