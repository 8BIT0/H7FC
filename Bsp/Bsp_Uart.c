/*
 * coder : 8_B!T0
 * bref : default using uart dma transfer
 */
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_uart.h"
#include "Bsp_Uart.h"

/* internal variable */
static UART_HandleTypeDef *BspUart_Handle_List[BspUART_Port_Sum] = {NULL};

/* external function */
static bool BspUart_Init(BspUARTObj_TypeDef *obj);

BspUART_TypeDef BspUart = {
    .de_init = NULL,
    .init = BspUart_Init,
};

static int BspUart_Init_Clock(BspUARTObj_TypeDef *obj)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    DMA_HandleTypeDef tx_dma_cfg = {0};
    DMA_HandleTypeDef rx_dma_cfg = {0};
    IRQn_Type irqn;
    int8_t index = Bspuart_None_Index;

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

    if ((obj == NULL) ||
        (obj->hdl.Instance == NULL) ||
        (obj->tx_dma_instance == NULL) ||
        (obj->rx_dma_instance == NULL))
        return BspUart_Clock_Error;

    if (obj->hdl.Instance == UART4)
    {
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART4;
        PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;

        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
            return BspUart_Clock_Error;

        __HAL_RCC_UART4_CLK_ENABLE();

        /* config */
        obj->tx_io.alternate = GPIO_AF8_UART4;
        obj->rx_io.alternate = GPIO_AF8_UART4;

        rx_dma_cfg.Init.Request = DMA_REQUEST_UART4_RX;
        tx_dma_cfg.Init.Request = DMA_REQUEST_UART4_TX;

        irqn = UART4_IRQn;
        index = BspUART_Port_4;
    }
    else if (obj->hdl.Instance == USART6)
    {
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART6;
        PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;

        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
            return BspUart_Clock_Error;

        __HAL_RCC_USART6_CLK_ENABLE();

        /* config */
        obj->tx_io.alternate = GPIO_AF7_USART6;
        obj->rx_io.alternate = GPIO_AF7_USART6;

        rx_dma_cfg.Init.Request = DMA_REQUEST_USART6_RX;
        tx_dma_cfg.Init.Request = DMA_REQUEST_USART6_TX;

        irqn = USART6_IRQn;
        index = BspUART_Port_6;
    }
    else if (obj->hdl.Instance == UART7)
    {
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART7;
        PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;

        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
            return BspUart_Clock_Error;

        __HAL_RCC_UART7_CLK_ENABLE();

        /* config */
        obj->tx_io.alternate = GPIO_AF7_UART7;
        obj->rx_io.alternate = GPIO_AF7_UART7;

        rx_dma_cfg.Init.Request = DMA_REQUEST_UART7_RX;
        tx_dma_cfg.Init.Request = DMA_REQUEST_UART7_TX;

        irqn = UART7_IRQn;
        index = BspUART_Port_7;
    }
    else
        return BspUart_Clock_Error;

    obj->tx_dma_hdl = tx_dma_cfg;
    obj->rx_dma_hdl = rx_dma_cfg;

    if (!BspDMA.regist(obj->rx_dma, obj->rx_stream, &(obj->rx_dma_hdl)) ||
        !BspDMA.regist(obj->tx_dma, obj->tx_stream, &(obj->rx_dma_hdl)))
        return BspUart_Clock_Error;

    /* rx tx pin init */
    BspGPIO.alt_init(obj->rx_io);
    BspGPIO.alt_init(obj->tx_io);

    if (HAL_DMA_Init(&rx_dma_cfg) != HAL_OK)
        return BspUart_Clock_Error;

    __HAL_LINKDMA(&(obj->hdl), hdmarx, rx_dma_cfg);

    if (HAL_DMA_Init(&tx_dma_cfg) != HAL_OK)
        return BspUart_Clock_Error;

    __HAL_LINKDMA(&(obj->hdl), hdmatx, tx_dma_cfg);

    HAL_NVIC_SetPriority(irqn, 0, 0);
    HAL_NVIC_EnableIRQ(irqn);

    return index;
}

static bool BspUart_Init(BspUARTObj_TypeDef *obj)
{
    UART_HandleTypeDef uart_cfg;
    int8_t port_index;

    port_index = BspUart_Init_Clock(obj);

    if(port_index < 0)
        return false;

    uart_cfg.Instance = obj->instance;
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

    obj->hdl = uart_cfg;

    /* swap tx rx pin */
    if (obj->pin_swap)
        uart_cfg.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;

    if (HAL_UART_Init(&uart_cfg) != HAL_OK ||
        HAL_UARTEx_SetTxFifoThreshold(&uart_cfg, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK ||
        HAL_UARTEx_SetRxFifoThreshold(&uart_cfg, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK ||
        HAL_UARTEx_DisableFifoMode(&uart_cfg) != HAL_OK)
        return false;

    /* enable irq callback */
    __HAL_UART_ENABLE_IT(&(obj->hdl), UART_IT_IDLE);
    __HAL_UART_DISABLE_IT(&(obj->hdl), UART_IT_ERR);
    __HAL_UART_DISABLE_IT(&(obj->hdl), UART_IT_PE);

    if ((obj->rx_buf == NULL) || (obj->rx_size == 0))
        return false;

    /* start receive data */
    HAL_UART_Receive_DMA(&(obj->hdl), obj->rx_buf, obj->rx_size);

    BspUart_Handle_List[port_index] = &(obj->hdl);
    return true;
}

static bool BspUart_Transfer(BspUARTObj_TypeDef *obj, uint8_t *tx_buf, uint16_t size)
{
    if ((obj == NULL) || (tx_buf == NULL) || (size == 0))
        return false;

    return true;
}

/******************************** irq callback ***********************************/
void USART1_Idle_Callback(void)
{
    uint32_t isrflags = READ_REG(huart1.Instance->ISR);
    uint32_t cr1its = READ_REG(huart1.Instance->CR1);

    if ((RESET != (isrflags & USART_ISR_IDLE)) && (RESET != (cr1its & USART_CR1_IDLEIE)))
    {
        uint16_t len;

        __HAL_UART_CLEAR_IDLEFLAG(&huart1);

        len = USART1_RXD_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
        if (0 != len)
        {
            const HAL_UART_StateTypeDef rxstate = huart1.RxState;

            /* Stop UART DMA Rx request if ongoing */
            if ((HAL_IS_BIT_SET(huart1.Instance->CR3, USART_CR3_DMAR)) &&
                (rxstate == HAL_UART_STATE_BUSY_RX))
            {
                CLEAR_BIT(huart1.Instance->CR3, USART_CR3_DMAR);

                /* Abort the UART DMA Rx channel */
                if (huart1.hdmarx != NULL)
                {
                    HAL_DMA_Abort(huart1.hdmarx);
                }

                /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
                CLEAR_BIT(huart1.Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
                CLEAR_BIT(huart1.Instance->CR3, USART_CR3_EIE);

                /* At end of Rx process, restore huart->RxState to Ready */
                huart1.RxState = HAL_UART_STATE_READY;
            }

            if (BspUsart1.receive_callback)
            {
                BspUsart1.receive_callback(USART1_Rx_Buffer, len);
            }
        }
        else
        {
            READ_REG(huart1.Instance->RDR);
            __HAL_UART_CLEAR_OREFLAG(&huart1);
        }
    }
}

void UART_Idle_Callback(BspUART_Port_List index)
{
    if (index == BspUART_Port_4)
    {
    }
    else if (index == BspUART_Port_6)
    {
    }
    else if (index == BspUART_Port_7)
    {
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
    }
    else if (huart->Instance == USART6)
    {
    }
    else if (huart->Instance == UART7)
    {
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)
    {
    }
    else if (huart->Instance == USART6)
    {
    }
    else if (huart->Instance == UART7)
    {
    }
}
