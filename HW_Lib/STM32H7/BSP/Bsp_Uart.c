/*
 * coder : 8_B!T0
 * bref : default using uart dma transfer
 */
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_uart.h"
#include "Bsp_Uart.h"

#define To_Uart_Instance(x) ((USART_TypeDef *)x)
#define To_Uart_Handle_Ptr(x) ((UART_HandleTypeDef *)x)
#define To_UartDMA_Handle_Ptr(x) ((DMA_HandleTypeDef *)x)

#define BspUart_Opr_TimeOut 100 // 100ms

/* internal variable */
static BspUARTObj_TypeDef *BspUart_Obj_List[BspUART_Port_Sum] = {NULL};

/* external function */
static bool BspUart_Init(BspUARTObj_TypeDef *obj);
static bool BspUart_Set_DataBit(BspUARTObj_TypeDef *obj, uint32_t bit);
static bool BspUart_Set_Parity(BspUARTObj_TypeDef *obj, uint32_t parity);
static bool BspUart_Set_StopBit(BspUARTObj_TypeDef *obj, uint32_t stop_bit);
static bool BspUart_Swap_Pin(BspUARTObj_TypeDef *obj, bool swap);
static bool BspUart_Transfer(BspUARTObj_TypeDef *obj, uint8_t *tx_buf, uint16_t size);
static bool BspUart_Set_Rx_Callback(BspUARTObj_TypeDef *obj, BspUART_Callback callback);
static bool BspUart_Set_Tx_Callback(BspUARTObj_TypeDef *obj, BspUART_Callback callback);

BspUART_TypeDef BspUart = {
    .init = BspUart_Init,
    .set_parity = BspUart_Set_Parity,
    .set_stop_bit = BspUart_Set_StopBit,
    .set_data_bit = BspUart_Set_DataBit,
    .set_swap = BspUart_Swap_Pin,
    .send = BspUart_Transfer,
    .set_rx_callback = BspUart_Set_Rx_Callback,
    .set_tx_callback = BspUart_Set_Tx_Callback,
};

static int BspUart_Init_Clock(BspUARTObj_TypeDef *obj)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    int8_t index = Bspuart_None_Index;

    if ((obj == NULL) ||
        (To_Uart_Instance(obj->instance) == NULL))
        return BspUart_Clock_Error;

    if (To_Uart_Instance(obj->instance) == USART1)
    {
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
        PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART1CLKSOURCE_D2PCLK2;

        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
            return BspUart_Clock_Error;

        __HAL_RCC_USART1_CLK_ENABLE();
        index = BspUART_Port_1;
    }
    else if (To_Uart_Instance(obj->instance) == UART4)
    {
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART4;
        PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;

        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
            return BspUart_Clock_Error;

        __HAL_RCC_UART4_CLK_ENABLE();
        index = BspUART_Port_4;
    }
    else if (To_Uart_Instance(obj->instance) == USART6)
    {
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART6;
        PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;

        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
            return BspUart_Clock_Error;

        __HAL_RCC_USART6_CLK_ENABLE();
        index = BspUART_Port_6;
    }
    else if (To_Uart_Instance(obj->instance) == UART7)
    {
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART7;
        PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;

        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
            return BspUart_Clock_Error;

        __HAL_RCC_UART7_CLK_ENABLE();
        index = BspUART_Port_7;
    }
    else
        return BspUart_Clock_Error;

    /* rx tx pin init */
    BspGPIO.alt_init(obj->rx_io, GPIO_MODE_AF_PP);
    BspGPIO.alt_init(obj->tx_io, GPIO_MODE_AF_PP);

    return index;
}

static int BspUart_Init_DMA(BspUARTObj_TypeDef *obj)
{
    DMA_HandleTypeDef tx_dma_cfg = {0};
    DMA_HandleTypeDef rx_dma_cfg = {0};
    int8_t index = Bspuart_None_Index;
    bool rx_dma_reg_state = false;
    bool tx_dma_reg_state = false;

    tx_dma_cfg.Init.PeriphInc = DMA_PINC_DISABLE;
    tx_dma_cfg.Init.MemInc = DMA_MINC_ENABLE;
    tx_dma_cfg.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    tx_dma_cfg.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    tx_dma_cfg.Init.Mode = DMA_NORMAL;
    tx_dma_cfg.Init.Priority = DMA_PRIORITY_MEDIUM;
    tx_dma_cfg.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    tx_dma_cfg.Init.Direction = DMA_MEMORY_TO_PERIPH;

    rx_dma_cfg.Init.PeriphInc = DMA_PINC_DISABLE;
    rx_dma_cfg.Init.MemInc = DMA_MINC_ENABLE;
    rx_dma_cfg.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    rx_dma_cfg.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    rx_dma_cfg.Init.Mode = DMA_NORMAL;
    rx_dma_cfg.Init.Priority = DMA_PRIORITY_MEDIUM;
    rx_dma_cfg.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    rx_dma_cfg.Init.Direction = DMA_PERIPH_TO_MEMORY;

    if ((obj == NULL) ||
        (To_Uart_Instance(obj->instance) == NULL))
        return BspUart_Clock_Error;

    if (To_Uart_Instance(obj->instance) == USART1)
    {
        rx_dma_cfg.Init.Request = DMA_REQUEST_USART1_RX;
        tx_dma_cfg.Init.Request = DMA_REQUEST_USART1_TX;

        index = BspUART_Port_1;
    }
    else if (To_Uart_Instance(obj->instance) == UART4)
    {
        rx_dma_cfg.Init.Request = DMA_REQUEST_UART4_RX;
        tx_dma_cfg.Init.Request = DMA_REQUEST_UART4_TX;

        index = BspUART_Port_4;
    }
    else if (To_Uart_Instance(obj->instance) == USART6)
    {
        rx_dma_cfg.Init.Request = DMA_REQUEST_USART6_RX;
        tx_dma_cfg.Init.Request = DMA_REQUEST_USART6_TX;

        index = BspUART_Port_6;
    }
    else if (To_Uart_Instance(obj->instance) == UART7)
    {
        rx_dma_cfg.Init.Request = DMA_REQUEST_UART7_RX;
        tx_dma_cfg.Init.Request = DMA_REQUEST_UART7_TX;

        index = BspUART_Port_7;
    }
    else
        return BspUart_Clock_Error;

    memcpy(obj->tx_dma_hdl, &tx_dma_cfg, sizeof(DMA_HandleTypeDef));
    memcpy(obj->rx_dma_hdl, &rx_dma_cfg, sizeof(DMA_HandleTypeDef));

    rx_dma_reg_state = BspDMA.regist(obj->rx_dma, obj->rx_stream, obj->rx_dma_hdl);
    tx_dma_reg_state = BspDMA.regist(obj->tx_dma, obj->tx_stream, obj->tx_dma_hdl);

    if (!rx_dma_reg_state && !tx_dma_reg_state)
        return BspUart_Clock_Error;

    if(rx_dma_reg_state)
    {
        if (HAL_DMA_Init(To_UartDMA_Handle_Ptr(obj->rx_dma_hdl)) != HAL_OK)
            return BspUart_Clock_Error;

        __HAL_LINKDMA(To_Uart_Handle_Ptr(obj->hdl), hdmarx, *To_UartDMA_Handle_Ptr(obj->rx_dma_hdl));

        BspDMA.enable_irq(obj->rx_dma, obj->rx_stream, 5, 0, 0, NULL);
    }

    if(tx_dma_reg_state)
    {
        if (HAL_DMA_Init(To_UartDMA_Handle_Ptr(obj->tx_dma_hdl)) != HAL_OK)
            return BspUart_Clock_Error;

        __HAL_LINKDMA(To_Uart_Handle_Ptr(obj->hdl), hdmatx, *To_UartDMA_Handle_Ptr(obj->tx_dma_hdl));

        BspDMA.enable_irq(obj->tx_dma, obj->tx_stream, 5, 0, 0, NULL);
    }

    return index;
}

static int BspUart_SetIRQ(BspUARTObj_TypeDef *obj)
{
    IRQn_Type irqn;
    int8_t index = Bspuart_None_Index;

    if ((obj == NULL) ||
        (To_Uart_Instance(obj->instance) == NULL))
        return BspUart_Clock_Error;

    if (To_Uart_Instance(obj->instance) == USART1)
    {
        irqn = USART1_IRQn;
        index = BspUART_Port_1;
    }
    else if (To_Uart_Instance(obj->instance) == UART4)
    {
        irqn = UART4_IRQn;
        index = BspUART_Port_4;
    }
    else if (To_Uart_Instance(obj->instance) == USART6)
    {
        irqn = USART6_IRQn;
        index = BspUART_Port_6;
    }
    else if (To_Uart_Instance(obj->instance) == UART7)
    {
        irqn = UART7_IRQn;
        index = BspUART_Port_7;
    }
    else
        return Bspuart_None_Index;

    HAL_NVIC_SetPriority(irqn, 5, 0);
    HAL_NVIC_EnableIRQ(irqn);

    return index;
}

static bool BspUart_Init(BspUARTObj_TypeDef *obj)
{
    UART_HandleTypeDef uart_cfg;
    int8_t port_index;

    obj->init_state = false;

    port_index = BspUart_Init_Clock(obj);

    if ((port_index < 0) || (To_Uart_Handle_Ptr(obj->hdl) == NULL))
        return false;

    uart_cfg.Instance = To_Uart_Instance(obj->instance);
    uart_cfg.Init.BaudRate = obj->baudrate;
    uart_cfg.Init.WordLength = UART_WORDLENGTH_8B;
    uart_cfg.Init.StopBits = UART_STOPBITS_1;
    uart_cfg.Init.Parity = UART_PARITY_NONE;
    uart_cfg.Init.Mode = UART_MODE_TX_RX;
    uart_cfg.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart_cfg.Init.OverSampling = UART_OVERSAMPLING_16;
    uart_cfg.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    uart_cfg.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    uart_cfg.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    uart_cfg.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;

    /* swap tx rx pin */
    if (obj->pin_swap)
        uart_cfg.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;

    memcpy(To_Uart_Handle_Ptr(obj->hdl), &uart_cfg, sizeof(UART_HandleTypeDef));

    if (HAL_UART_Init(To_Uart_Handle_Ptr(obj->hdl)) != HAL_OK)
        return false;

    if (BspUart_Init_DMA(obj) < 0)
    {
        obj->irq_type = BspUart_IRQ_Type_Byte;
    }
    else
        obj->irq_type = BspUart_IRQ_Type_Idle;

    uint32_t tick = HAL_GetTick();
    while(HAL_GetTick() - tick < 50);

    if (HAL_UARTEx_SetTxFifoThreshold(&uart_cfg, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK ||
        HAL_UARTEx_SetRxFifoThreshold(&uart_cfg, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK ||
        HAL_UARTEx_DisableFifoMode(&uart_cfg) != HAL_OK)
        return false;

    if ((obj->rx_buf == NULL) || (obj->rx_size == 0))
        return false;

    BspUart_Obj_List[port_index] = obj;

    memset(&(obj->monitor), 0, sizeof(obj->monitor));

    /* enable irq callback */
    switch (obj->irq_type)
    {
        case BspUart_IRQ_Type_Idle:
            __HAL_UART_ENABLE_IT(To_Uart_Handle_Ptr(obj->hdl), UART_IT_IDLE);
            break;

        case BspUart_IRQ_Type_Byte:
            __HAL_UART_ENABLE_IT(To_Uart_Handle_Ptr(obj->hdl), UART_IT_RXNE | UART_IT_ERR | UART_IT_ORE);
            break;

        default:
            return false;
    }

    __HAL_UART_DISABLE_IT(To_Uart_Handle_Ptr(obj->hdl), UART_IT_ERR);
    __HAL_UART_DISABLE_IT(To_Uart_Handle_Ptr(obj->hdl), UART_IT_PE);

    if (BspUart_SetIRQ(obj) < 0)
        return false;

    if (obj->irq_type == BspUart_IRQ_Type_Idle)
    {
        /* start dma receive data */
        HAL_UART_Receive_DMA(To_Uart_Handle_Ptr(obj->hdl), obj->rx_buf, obj->rx_size);
    }
    else if (obj->irq_type == BspUart_IRQ_Type_Byte)
    {
        /* start receive irq */
        HAL_UART_Receive_IT(To_Uart_Handle_Ptr(obj->hdl), &obj->rx_single_byte, 1);
    }

    obj->init_state = true;
    return true;
}

static bool BspUart_Set_DataBit(BspUARTObj_TypeDef *obj, uint32_t bit)
{
    if (obj || !obj->init_state)
        return false;

    To_Uart_Handle_Ptr(obj->hdl)->Init.WordLength = bit;

    if (HAL_UART_Init(To_Uart_Handle_Ptr(obj->hdl)) != HAL_OK)
        return false;

    return true;
}

static bool BspUart_Set_Parity(BspUARTObj_TypeDef *obj, uint32_t parity)
{
    if (obj || !obj->init_state)
        return false;

    To_Uart_Handle_Ptr(obj->hdl)->Init.Parity = parity;

    if (HAL_UART_Init(To_Uart_Handle_Ptr(obj->hdl)) != HAL_OK)
        return false;

    return true;
}

static bool BspUart_Set_StopBit(BspUARTObj_TypeDef *obj, uint32_t stop_bit)
{
    if (obj || !obj->init_state)
        return false;

    To_Uart_Handle_Ptr(obj->hdl)->Init.StopBits = stop_bit;

    if (HAL_UART_Init(To_Uart_Handle_Ptr(obj->hdl)) != HAL_OK)
        return false;

    return true;
}

static bool BspUart_Swap_Pin(BspUARTObj_TypeDef *obj, bool swap)
{
    if (obj || !obj->init_state)
        return false;

    obj->pin_swap = swap;

    if (swap)
    {
        To_Uart_Handle_Ptr(obj->hdl)->AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
    }
    else
        To_Uart_Handle_Ptr(obj->hdl)->AdvancedInit.Swap = UART_ADVFEATURE_SWAP_DISABLE;

    if (HAL_UART_Init(To_Uart_Handle_Ptr(obj->hdl)) != HAL_OK)
        return false;

    return true;
}

UART_HandleTypeDef *BspUart_GetObj_Handle(BspUART_Port_List index)
{
    if (index >= BspUART_Port_Sum)
        return NULL;

    return &(BspUart_Obj_List[index]->hdl);
}

static bool BspUart_Transfer(BspUARTObj_TypeDef *obj, uint8_t *tx_buf, uint16_t size)
{
    uint8_t time_out = BspUart_Opr_TimeOut;

    if ((obj == NULL) || (tx_buf == NULL) || (size == 0))
        return false;

    /* last pack still in transmitting */
    if(obj->monitor.tx_success_cnt != obj->monitor.tx_cnt)
        return false;

    /* send data */
    switch (HAL_UART_Transmit_DMA(To_Uart_Handle_Ptr(obj->hdl), tx_buf, size))
    {
        case HAL_OK:
            obj->monitor.tx_cnt++;
            break;

        case HAL_ERROR:
        case HAL_BUSY:
            obj->monitor.tx_err_cnt++;
            return false;

        default:
            obj->monitor.tx_unknow_err_cnt++;
            return false;
    }

    return true;
}

static void BspUart_DMAStopRx(BspUART_Port_List index)
{
    UART_HandleTypeDef hdl;

    if (BspUart_Obj_List[index])
    {
        memcpy(&hdl, BspUart_Obj_List[index]->hdl, sizeof(UART_HandleTypeDef));

        const HAL_UART_StateTypeDef rxstate = hdl.RxState;

        /* Stop UART DMA Rx request if ongoing */
        if ((HAL_IS_BIT_SET(hdl.Instance->CR3, USART_CR3_DMAR)) &&
            (rxstate == HAL_UART_STATE_BUSY_RX))
        {
            CLEAR_BIT(hdl.Instance->CR3, USART_CR3_DMAR);

            /* Abort the UART DMA Rx channel */
            if (hdl.hdmarx != NULL)
            {
                HAL_DMA_Abort(hdl.hdmarx);
            }

            /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
            CLEAR_BIT(hdl.Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
            CLEAR_BIT(hdl.Instance->CR3, USART_CR3_EIE);

            /* At end of Rx process, restore huart->RxState to Ready */
            hdl.RxState = HAL_UART_STATE_READY;
        }
    }
}

static bool BspUart_Set_Rx_Callback(BspUARTObj_TypeDef *obj, BspUART_Callback callback)
{
    if(obj)
    {
        obj->RxCallback = callback;
        return true;
    }

    return false;
}

static bool BspUart_Set_Tx_Callback(BspUARTObj_TypeDef *obj, BspUART_Callback callback)
{
    if(obj)
    {
        obj->TxCallback = callback;
        return true;
    }

    return false;
}

/******************************** irq callback ***********************************/
void UART_IRQ_Callback(BspUART_Port_List index)
{
    static UART_HandleTypeDef *hdl = NULL;
    static DMA_HandleTypeDef *rx_dma = NULL;
    uint16_t len = 0;
    uint32_t isrflags = 0;
    uint32_t cr1its = 0;
    uint32_t errorflags = 0;
    uint32_t cr3its = 0;

    if (BspUart_Obj_List[index] && BspUart_Obj_List[index]->irq_type == BspUart_IRQ_Type_Idle)
    {

        hdl = BspUart_Obj_List[index]->hdl;
        isrflags = READ_REG(hdl->Instance->ISR);
        cr1its = READ_REG(hdl->Instance->CR1);
        cr3its = READ_REG(hdl->Instance->CR3);
        errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE | USART_ISR_RTOF));

        if (errorflags != 0)
            return;

        rx_dma = To_UartDMA_Handle_Ptr(BspUart_Obj_List[index]->rx_dma_hdl);

        if (hdl && rx_dma)
        {
            const HAL_UART_StateTypeDef rxstate = hdl->RxState;

            if ((RESET != (isrflags & USART_ISR_IDLE)) && (RESET != (cr1its & USART_CR1_IDLEIE)))
            {
                __HAL_UART_CLEAR_IDLEFLAG(hdl);

                BspUart_DMAStopRx(index);

                len = BspUart_Obj_List[index]->rx_size - __HAL_DMA_GET_COUNTER(rx_dma);

                if (len)
                {
                    /* idle receive callback process */
                    if (BspUart_Obj_List[index]->RxCallback)
                        BspUart_Obj_List[index]->RxCallback(BspUart_Obj_List[index]->cust_data_addr,
                                                            BspUart_Obj_List[index]->rx_buf,
                                                            len);

                    BspUart_Obj_List[index]->monitor.rx_cnt++;
                    HAL_UART_Receive_DMA(hdl, BspUart_Obj_List[index]->rx_buf, BspUart_Obj_List[index]->rx_size);
                }
                else
                {
                    BspUart_Obj_List[index]->monitor.rx_err_cnt++;
                    READ_REG(hdl->Instance->RDR);
                    __HAL_UART_CLEAR_OREFLAG(hdl);
                }
            }
        }
    }
}

/* receive full */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t index = 0;

    if (huart->Instance == USART1)
    {
        index = BspUART_Port_1;
    }
    else if (huart->Instance == UART4)
    {
        index = BspUART_Port_4;
    }
    else if (huart->Instance == USART6)
    {
        index = BspUART_Port_6;
    }
    else if (huart->Instance == UART7)
    {
        index = BspUART_Port_7;
    }
    else
        return;

    if (BspUart_Obj_List[index])
    {
        if (BspUart_Obj_List[index]->irq_type == BspUart_IRQ_Type_Idle)
        {
            if (BspUart_Obj_List[index]->RxCallback)
                BspUart_Obj_List[index]->RxCallback(BspUart_Obj_List[index]->cust_data_addr,
                                                    BspUart_Obj_List[index]->rx_buf,
                                                    BspUart_Obj_List[index]->rx_size);

            HAL_UART_Receive_DMA(&(BspUart_Obj_List[index]->hdl), BspUart_Obj_List[index]->rx_buf, BspUart_Obj_List[index]->rx_size);

            BspUart_Obj_List[index]->monitor.rx_full_cnt++;
        }
        else if (BspUart_Obj_List[index]->irq_type == BspUart_IRQ_Type_Byte)
        {
            if (BspUart_Obj_List[index]->RxCallback)
                BspUart_Obj_List[index]->RxCallback(BspUart_Obj_List[index]->cust_data_addr, &BspUart_Obj_List[index]->rx_single_byte, 1);

            HAL_UART_Receive_IT(&(BspUart_Obj_List[index]->hdl), &BspUart_Obj_List[index]->rx_single_byte, 1);
        }
    }
}

/* transfer full */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t index = 0;

    if(huart == NULL)
        return;

    if(huart->Instance == USART1)
    {
        index = BspUART_Port_1;
    }
    else if (huart->Instance == UART4)
    {
        index = BspUART_Port_4;
    }
    else if (huart->Instance == USART6)
    {
        index = BspUART_Port_6;
    }
    else if (huart->Instance == UART7)
    {
        index = BspUART_Port_7;
    }
    else
        return;

    if (BspUart_Obj_List[index])
    {
        BspUart_Obj_List[index]->monitor.tx_success_cnt ++;

        if(BspUart_Obj_List[index]->TxCallback)
            BspUart_Obj_List[index]->TxCallback(BspUart_Obj_List[index]->cust_data_addr, NULL, 0);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint8_t index = 0;
    
    if(huart == NULL)
        return;

    if(huart->Instance == USART1)
    {
        index = BspUART_Port_1;
    }
    else if (huart->Instance == UART4)
    {
        index = BspUART_Port_4;
    }
    else if (huart->Instance == USART6)
    {
        index = BspUART_Port_6;
    }
    else if (huart->Instance == UART7)
    {
        index = BspUART_Port_7;
    }
    else
        return;
 
    if (BspUart_Obj_List[index] && huart->ErrorCode)
    {
        BspUart_Obj_List[index]->monitor.rx_err_cnt ++;
        
        if(BspUart_Obj_List[index]->irq_type == BspUart_IRQ_Type_Byte)
        {
            if(huart->ErrorCode && HAL_UART_ERROR_ORE)
            {
                BspUart_Obj_List[index]->monitor.ore_cnt ++;

                uint8_t temp = huart->Instance->RDR;
                HAL_UART_Receive_IT(&(BspUart_Obj_List[index]->hdl), &BspUart_Obj_List[index]->rx_single_byte, 1);
            }
        }
    }
}

