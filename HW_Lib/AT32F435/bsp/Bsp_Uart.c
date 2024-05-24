#include "Bsp_Uart.h"
#include "Bsp_GPIO.h"
#include "Bsp_DMA.h"
#include "at32f435_437_usart.h"

#define Uart_Default_Baudrate 460800

#define Uart_Dir_Tx 0
#define Uart_Dir_Rx 1

#define To_Uart_Instance(x) ((usart_type *)x)
#define To_UartObj_Ptr(x) ((BspUARTObj_TypeDef *)x)

#define BspUart_Opr_TimeOut 100 // 100ms

/* internal variable */
static BspUARTObj_TypeDef *BspUart_Obj_List[BspUART_Port_Sum] = {NULL};
static BspDMA_IrqCall_Obj_TypeDef BspUart_TxDMA_IrqObj[BspUART_Port_Sum] = {{NULL, NULL}};
static BspDMA_IrqCall_Obj_TypeDef BspUart_RxDMA_IrqObj[BspUART_Port_Sum] = {{NULL, NULL}};

/* internal function */
static int BspUart_Init_Clock(BspUARTObj_TypeDef *obj);
static int BspUart_SetIRQ(BspUARTObj_TypeDef *obj, bool state);
static dmamux_requst_id_sel_type BspUart_Get_MuxSeq(usart_type *port, uint8_t dir, void *dma_hdl);
static int BspUart_Init_DMA(BspUARTObj_TypeDef *obj);
static void BspUart_DMA_TxCplt_Callback(void *arg);
static void BspUart_DMA_RxCplt_Callback(void *arg);

/* external function */
static bool BspUart_Init(BspUARTObj_TypeDef *obj);
static bool BspUart_DeInit(BspUARTObj_TypeDef *obj);
static bool BspUart_Set_DataBit(BspUARTObj_TypeDef *obj, uint32_t bit);
static bool BspUart_Set_Parity(BspUARTObj_TypeDef *obj, uint32_t parity);
static bool BspUart_Set_StopBit(BspUARTObj_TypeDef *obj, uint32_t stop_bit);
static bool BspUart_Swap_Pin(BspUARTObj_TypeDef *obj, bool swap);
static bool BspUart_Transfer(BspUARTObj_TypeDef *obj, uint8_t *tx_buf, uint16_t size);
static bool BspUart_Set_Rx_Callback(BspUARTObj_TypeDef *obj, BspUART_Callback callback);
static bool BspUart_Set_Tx_Callback(BspUARTObj_TypeDef *obj, BspUART_Callback callback);

BspUART_TypeDef BspUart = {
    .init = BspUart_Init,
    .de_init = BspUart_DeInit,
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
    int index = 0;
    crm_periph_clock_type clock = CRM_USART1_PERIPH_CLOCK;

    if((obj == NULL) || (obj->instance == NULL))
        return BspUart_Clock_Error;

    if(To_Uart_Instance(obj->instance) == USART1)
    {
        clock = CRM_USART1_PERIPH_CLOCK;
        index = BspUART_Port_1;
    }
    else if(To_Uart_Instance(obj->instance) == USART2)
    {
        clock = CRM_USART2_PERIPH_CLOCK;
        index = BspUART_Port_2;
    }
    else if(To_Uart_Instance(obj->instance) == USART3)
    {
        clock = CRM_USART3_PERIPH_CLOCK;
        index = BspUART_Port_3;
    }
    else if(To_Uart_Instance(obj->instance) == UART4)
    {
        clock = CRM_UART4_PERIPH_CLOCK;
        index = BspUART_Port_4;
    }
    else if(To_Uart_Instance(obj->instance) == UART5)
    {
        clock = CRM_UART5_PERIPH_CLOCK;
        index = BspUART_Port_5;
    }
    else if(To_Uart_Instance(obj->instance) == USART6)
    {
        clock = CRM_USART6_PERIPH_CLOCK;
        index = BspUART_Port_6;
    }
    else if(To_Uart_Instance(obj->instance) == UART7)
    {
        clock = CRM_UART7_PERIPH_CLOCK;
        index = BspUART_Port_7;
    }
    else if(To_Uart_Instance(obj->instance) == UART8)
    {
        clock = CRM_UART8_PERIPH_CLOCK;
        index = BspUART_Port_8;
    }
    else
        return BspUart_Clock_Error;

    crm_periph_clock_enable(clock, TRUE);
    return index;
}

static int BspUart_SetIRQ(BspUARTObj_TypeDef *obj, bool state)
{
    int index = 0;
    IRQn_Type irqn = BspUART_Port_1;

    if((obj == NULL) || (obj->instance == NULL))
        return BspUart_Clock_Error;

    if(To_Uart_Instance(obj->instance) == USART1)
    {
        irqn = USART1_IRQn;
        index = BspUART_Port_1;
    }
    else if(To_Uart_Instance(obj->instance) == USART2)
    {
        irqn = USART2_IRQn;
        index = BspUART_Port_2;
    }
    else if(To_Uart_Instance(obj->instance) == USART3)
    {
        irqn = USART3_IRQn;
        index = BspUART_Port_3;
    }
    else if(To_Uart_Instance(obj->instance) == UART4)
    {
        irqn = UART4_IRQn;
        index = BspUART_Port_4;
    }
    else if(To_Uart_Instance(obj->instance) == UART5)
    {
        irqn = UART5_IRQn;
        index = BspUART_Port_5;
    }
    else if(To_Uart_Instance(obj->instance) == USART6)
    {
        irqn = USART6_IRQn;
        index = BspUART_Port_6;
    }
    else if(To_Uart_Instance(obj->instance) == UART7)
    {
        irqn = UART7_IRQn;
        index = BspUART_Port_7;
    }
    else if(To_Uart_Instance(obj->instance) == UART8)
    {
        irqn = UART8_IRQn;
        index = BspUART_Port_7;
    }

    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
    if (state)
    {
        nvic_irq_enable(irqn, 5, 0);
    }
    else
        nvic_irq_disable(irqn);

    return index;
}

static dmamux_requst_id_sel_type BspUart_Get_MuxSeq(usart_type *port, uint8_t dir, void *dma_hdl)
{
    if ((port == NULL) || (dma_hdl == NULL))
        return 0;

    if (port == USART1)
    {
        if (dir == Uart_Dir_Rx)
            return DMAMUX_DMAREQ_ID_USART1_RX;
        
        if (dir == Uart_Dir_Tx)
            return DMAMUX_DMAREQ_ID_USART1_TX;
    }
    else if (port == USART2)
    {
        if(dir == Uart_Dir_Rx)
            return DMAMUX_DMAREQ_ID_USART2_RX;
        
        if(dir == Uart_Dir_Tx)
            return DMAMUX_DMAREQ_ID_USART2_TX;
    }
    else if (port == USART3)
    {
        if (dir == Uart_Dir_Rx)
            return DMAMUX_DMAREQ_ID_USART3_RX;

        if (dir == Uart_Dir_Tx)
            return DMAMUX_DMAREQ_ID_USART3_TX;
    }
    else if (port == UART4)
    {
        if (dir == Uart_Dir_Rx)
            return DMAMUX_DMAREQ_ID_UART4_RX;
        
        if (dir == Uart_Dir_Tx)
            return DMAMUX_DMAREQ_ID_UART4_TX;
    }
    else if (port == UART5)
    {
        if(dir == Uart_Dir_Rx)
            return DMAMUX_DMAREQ_ID_UART5_RX;
        
        if(dir == Uart_Dir_Tx)
            return DMAMUX_DMAREQ_ID_UART5_TX;
    }
    else if (port == USART6)
    {
        if (dir == Uart_Dir_Rx)
            return DMAMUX_DMAREQ_ID_USART6_RX;

        if (dir == Uart_Dir_Tx)
            return DMAMUX_DMAREQ_ID_USART6_TX;
    }
    else if (port == UART7)
    {
        if (dir == Uart_Dir_Rx)
            return DMAMUX_DMAREQ_ID_UART7_RX;
        
        if (dir == Uart_Dir_Tx)
            return DMAMUX_DMAREQ_ID_UART7_TX;
    }
    else if (port == UART8)
    {
        if (dir == Uart_Dir_Rx)
            return DMAMUX_DMAREQ_ID_UART8_RX;
        
        if (dir == Uart_Dir_Tx)
            return DMAMUX_DMAREQ_ID_UART8_TX;
    }

    return 0;
}

static int BspUart_Init_DMA(BspUARTObj_TypeDef *obj)
{
    int index = 0;
    dma_init_type tx_dma;
    dma_init_type rx_dma;

    if ((obj == NULL) || \
        (obj->instance == NULL))
        return Bspuart_None_Index;

    if (To_Uart_Instance(obj->instance) == USART1)
    {
        index = BspUART_Port_1;
    }
    else if (To_Uart_Instance(obj->instance) == USART2)
    {
        index = BspUART_Port_2;
    }
    else if (To_Uart_Instance(obj->instance) == USART3)
    {
        index = BspUART_Port_3;
    }
    else if (To_Uart_Instance(obj->instance) == UART4)
    {
        index = BspUART_Port_4;
    }
    else if (To_Uart_Instance(obj->instance) == UART5)
    {
        index = BspUART_Port_5;
    }
    else if (To_Uart_Instance(obj->instance) == USART6)
    {
        index = BspUART_Port_6;
    }
    else if (To_Uart_Instance(obj->instance) == UART7)
    {
        index = BspUART_Port_7;
    }
    else if (To_Uart_Instance(obj->instance) == UART8)
    {
        index = BspUART_Port_8;
    }
    else
        return Bspuart_None_Index;

    /* rx dma init section */
    obj->rx_dma_hdl = BspDMA.get_instance(obj->rx_dma, obj->rx_stream);
    if(obj->rx_dma_hdl)
    {
        if ((obj->rx_buf == NULL) || (obj->rx_size == 0))
            return BspUart_Clock_Error;

        dma_reset(obj->rx_dma_hdl);
        dma_default_para_init(&rx_dma);

        rx_dma.buffer_size = obj->rx_size;
        rx_dma.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
        rx_dma.memory_base_addr = (uint32_t)obj->rx_buf;
        rx_dma.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
        rx_dma.memory_inc_enable = TRUE;
        rx_dma.peripheral_base_addr = (uint32_t)&(To_Uart_Instance(obj->instance)->dt);
        rx_dma.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
        rx_dma.peripheral_inc_enable = FALSE;
        rx_dma.priority = DMA_PRIORITY_HIGH;
        rx_dma.loop_mode_enable = FALSE;

        BspUart_RxDMA_IrqObj[index].BspDMA_Irq_Callback_Func = BspUart_DMA_RxCplt_Callback;
        BspUart_RxDMA_IrqObj[index].cus_data = (void *)obj;

        dma_init(obj->rx_dma_hdl, &rx_dma);
        BspDMA.enable_irq(obj->rx_dma, obj->rx_stream, 5, 0, \
                          BspUart_Get_MuxSeq(To_Uart_Instance(obj->instance), \
                          Uart_Dir_Rx, obj->rx_dma_hdl), \
                          (void *)&BspUart_RxDMA_IrqObj[index]);
    }

    /* tx dma init section */
    obj->tx_dma_hdl = BspDMA.get_instance(obj->tx_dma, obj->tx_stream);
    if(obj->tx_dma_hdl)
    {
        dma_reset(obj->tx_dma_hdl);
        dma_default_para_init(&tx_dma);

        tx_dma.buffer_size = 0;
        tx_dma.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
        tx_dma.memory_base_addr = (uint32_t)0;
        tx_dma.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
        tx_dma.memory_inc_enable = TRUE;
        tx_dma.peripheral_base_addr = (uint32_t)&(To_Uart_Instance(obj->instance)->dt);
        tx_dma.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
        tx_dma.peripheral_inc_enable = FALSE;
        tx_dma.priority = DMA_PRIORITY_HIGH;
        tx_dma.loop_mode_enable = FALSE;

        BspUart_TxDMA_IrqObj[index].BspDMA_Irq_Callback_Func = BspUart_DMA_TxCplt_Callback;
        BspUart_TxDMA_IrqObj[index].cus_data = (void *)obj;

        dma_init(obj->tx_dma_hdl, &tx_dma);
        
        BspDMA.enable_irq(obj->tx_dma, obj->tx_stream, 5, 0, \
                          BspUart_Get_MuxSeq(To_Uart_Instance(obj->instance), \
                          Uart_Dir_Tx, obj->tx_dma_hdl), \
                          &BspUart_TxDMA_IrqObj[index]);
    }

    return index;
}

static bool BspUart_Init(BspUARTObj_TypeDef *obj)
{
    int port_index = -1;

    if ((obj == NULL) || \
        (obj->instance == NULL))
        return false;

    port_index = BspUart_Init_Clock(obj);

    if(port_index < 0)
        return false;

    /* pin init */
    if (!BspGPIO.alt_init(obj->tx_io, 0))
        return false;

    if (!BspGPIO.alt_init(obj->rx_io, 0))
        return false;

    if(obj->baudrate <= 0)
        obj->baudrate = Uart_Default_Baudrate;

    usart_init(To_Uart_Instance(obj->instance), obj->baudrate, USART_DATA_8BITS, USART_STOP_1_BIT);
    
    if (obj->pin_swap)
        usart_transmit_receive_pin_swap(To_Uart_Instance(obj->instance), TRUE);
    
    usart_transmitter_enable(To_Uart_Instance(obj->instance), TRUE);
    usart_receiver_enable(To_Uart_Instance(obj->instance), TRUE);

    if(BspUart_Init_DMA(obj) == BspUart_Clock_Error)
        return false;

    /* irq config */
    if(obj->rx_dma_hdl)
    {
        usart_dma_receiver_enable(To_Uart_Instance(obj->instance), TRUE);
        usart_interrupt_enable(To_Uart_Instance(obj->instance), USART_IDLE_INT, TRUE);
        obj->irq_type = BspUart_IRQ_Type_Idle;
    }
    else
    {
        usart_interrupt_enable(To_Uart_Instance(obj->instance), USART_RDBF_INT, TRUE);
        obj->irq_type = BspUart_IRQ_Type_Byte;
    }

    if(obj->tx_dma_hdl)
        usart_dma_transmitter_enable(To_Uart_Instance(obj->instance), TRUE);
    usart_interrupt_enable(To_Uart_Instance(obj->instance), USART_TDBE_INT, FALSE);

    usart_enable(To_Uart_Instance(obj->instance), TRUE);
    if(BspUart_SetIRQ(obj, true) < 0)
        return false;
    
    dma_channel_enable(To_DMA_Handle_Ptr(obj->rx_dma_hdl), TRUE);

    BspUart_Obj_List[port_index] = obj;
    obj->init_state = true;
    
    return true;
}

static bool BspUart_DeInit(BspUARTObj_TypeDef *obj)
{
    if (obj && obj->instance)
    {
        /* deinit dma port */
        if (obj->rx_dma_hdl)
        {
            dma_channel_enable(To_DMA_Handle_Ptr(obj->rx_dma_hdl), FALSE);
            dma_reset(To_DMA_Handle_Ptr(obj->rx_dma_hdl));
        }

        if (obj->tx_dma_hdl)
        {
            dma_channel_enable(To_DMA_Handle_Ptr(obj->tx_dma_hdl), FALSE);
            dma_reset(To_DMA_Handle_Ptr(obj->tx_dma_hdl));
        }

        if (obj->irq_type == BspUart_IRQ_Type_Idle)
        {
            usart_interrupt_enable(To_Uart_Instance(obj->instance), USART_IDLE_INT, FALSE);
        }
        else if (obj->irq_type == BspUart_IRQ_Type_Byte)
            usart_interrupt_enable(To_Uart_Instance(obj->instance), USART_RDBF_INT, FALSE);

        BspUart_SetIRQ(obj, false);

        BspGPIO.de_init(obj->rx_io);
        BspGPIO.de_init(obj->tx_io);

        /* deinit uart port */
        usart_reset(To_Uart_Instance(obj->instance));
    }

    return false;
}

static bool BspUart_Set_DataBit(BspUARTObj_TypeDef *obj, uint32_t bit)
{
    usart_identification_bit_num_type bit_num = (usart_identification_bit_num_type)bit; 

    if(obj && obj->instance && obj->init_state && (bit_num <= USART_ID_RELATED_DATA_BIT))
    {
        usart_id_bit_num_set(To_Uart_Instance(obj->instance), bit_num);
        return true;
    }

    return false;
}

static bool BspUart_Set_Parity(BspUARTObj_TypeDef *obj, uint32_t parity)
{
    usart_parity_selection_type parity_tmp = (usart_parity_selection_type)parity;

    if (obj && obj->instance && obj->init_state && (parity_tmp <= USART_PARITY_ODD))
    {
        usart_parity_selection_config(To_Uart_Instance(obj->instance), parity_tmp);
        return true;
    }

    return false;
}

static bool BspUart_Set_StopBit(BspUARTObj_TypeDef *obj, uint32_t stop_bit)
{
    return false;
}

static bool BspUart_Swap_Pin(BspUARTObj_TypeDef *obj, bool swap)
{
    if(obj && obj->instance)
    {
        obj->pin_swap = true;
        
        if(obj->init_state)
            usart_transmit_receive_pin_swap(To_Uart_Instance(obj->instance), TRUE);
    }

    return true;
}

static bool BspUart_Transfer(BspUARTObj_TypeDef *obj, uint8_t *tx_buf, uint16_t size)
{
    if(obj && obj->instance && obj->init_state && tx_buf && size)
    {
        /* if has dma handle use dma trans */
        if(obj->tx_dma_hdl)
        {
            /* last pack still in transfer */
            if(obj->monitor.tx_success_cnt != obj->monitor.tx_cnt)
                return false;

            To_DMA_Handle_Ptr(obj->tx_dma_hdl)->maddr = (uint32_t)tx_buf;
            To_DMA_Handle_Ptr(obj->tx_dma_hdl)->dtcnt_bit.cnt = size;

            dma_channel_enable(To_DMA_Handle_Ptr(obj->tx_dma_hdl), TRUE);
        }
        else
        {
            for (uint16_t i = 0; i < size; i++)
                usart_data_transmit(To_Uart_Instance(obj->instance), tx_buf[i]);
        }
        obj->monitor.tx_cnt ++;

        return true;
    }

    return false;
}

static bool BspUart_Set_Rx_Callback(BspUARTObj_TypeDef *obj, BspUART_Callback callback)
{
    if (obj && obj->instance && obj->init_state)
    {
        obj->RxCallback = callback;
        return true;
    }

    return false;
}

static bool BspUart_Set_Tx_Callback(BspUARTObj_TypeDef *obj, BspUART_Callback callback)
{
    if (obj && obj->instance && obj->init_state)
    {
        obj->TxCallback = callback;
        return true;
    }

    return false;
}

/* dma tx finish */
static void BspUart_DMA_TxCplt_Callback(void *arg)
{
    usart_type *instance = NULL;
    BspUARTObj_TypeDef *obj = NULL;

    if(arg)
    {
        obj = To_UartObj_Ptr(arg);
        instance = To_Uart_Instance(obj->instance);
        
        if(instance == NULL)
            return;

        if(obj->tx_dma_hdl)
        {
            dma_channel_enable(To_DMA_Handle_Ptr(obj->tx_dma_hdl), FALSE);
            obj->monitor.tx_success_cnt ++;

            if(obj->TxCallback)
                obj->TxCallback(obj->cust_data_addr, NULL, 0);
        }
        else
        {
            obj->monitor.tx_err_cnt ++;
        }
    }
}

/* dma rx full */
static void BspUart_DMA_RxCplt_Callback(void *arg)
{
    usart_type *instance = NULL;
    BspUARTObj_TypeDef *obj = NULL;
    uint32_t dma_rec_size = 0;
    
    if(arg)
    {
        obj = To_UartObj_Ptr(arg);
        instance = To_Uart_Instance(obj->instance);

        if(instance == NULL)
            return;

        obj->monitor.rx_full_cnt ++;

        if(obj->rx_dma_hdl)
        {
            dma_channel_enable(To_DMA_Handle_Ptr(obj->rx_dma_hdl), FALSE);
            dma_rec_size = obj->rx_size - dma_data_number_get(To_DMA_Handle_Ptr(obj->rx_dma_hdl));

            /* if full size unequal to uart dma port set max receive size then current receive is error */
            if(dma_rec_size != obj->rx_size)
                obj->monitor.rx_err_cnt ++;

            if(obj->RxCallback)
                obj->RxCallback(obj->cust_data_addr, obj->rx_buf, dma_rec_size);
        }
        else
        {
            obj->monitor.rx_err_cnt ++;
        }
    }
}

void BspUart_Irq_Callback(void *arg)
{
    usart_type *instance = NULL;
    BspUARTObj_TypeDef *obj = NULL;
    volatile uint16_t dma_rec_size = 0;
    uint8_t rx_byte = 0;

    if (arg)
    {
        instance = To_Uart_Instance(arg);
        usart_data_receive(instance);
    
        if (instance == USART1)
            obj = BspUart_Obj_List[BspUART_Port_1];

        if (instance == USART2)
            obj = BspUart_Obj_List[BspUART_Port_2];
        
        if (instance == USART3)
            obj = BspUart_Obj_List[BspUART_Port_3];

        if (instance == UART4)
            obj = BspUart_Obj_List[BspUART_Port_4];
        
        if (instance == UART5)
            obj = BspUart_Obj_List[BspUART_Port_5];
        
        if (instance == USART6)
            obj = BspUart_Obj_List[BspUART_Port_6];
        
        if (instance == UART7)
            obj = BspUart_Obj_List[BspUART_Port_7];
        
        if (instance == UART8)
            obj = BspUart_Obj_List[BspUART_Port_8];
    
        if (obj && obj->instance && obj->init_state && (obj->instance == instance))
        {
            if ((obj->irq_type == BspUart_IRQ_Type_Idle) && obj->rx_dma_hdl)
            {
                dma_channel_enable(To_DMA_Handle_Ptr(obj->rx_dma_hdl), FALSE);

                dma_rec_size = obj->rx_size - dma_data_number_get(obj->rx_dma_hdl);
                
                obj->monitor.rx_cnt ++;
                if(obj->RxCallback)
                    obj->RxCallback(obj->cust_data_addr, obj->rx_buf, dma_rec_size);

                To_DMA_Handle_Ptr(obj->rx_dma_hdl)->maddr = (uint32_t)obj->rx_buf;
                To_DMA_Handle_Ptr(obj->rx_dma_hdl)->dtcnt = obj->rx_size;
                dma_channel_enable(To_DMA_Handle_Ptr(obj->rx_dma_hdl), TRUE);
            }
            else if (obj->irq_type == BspUart_IRQ_Type_Byte)
            {
                rx_byte = usart_data_receive(To_Uart_Instance(obj->instance));
                if(obj->RxCallback)
                    obj->RxCallback(obj->cust_data_addr, &rx_byte, 1);
            }
        }
    }
}
