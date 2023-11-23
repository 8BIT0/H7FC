#include "Bsp_USB.h"

typedef enum
{
    BspUSB_VCP_TxSrc_Queue = 0,
    BspUSB_VCP_TxSrc_Input,
}BspUSB_VCP_TxSource_List;

static BspUSB_VCP_Obj_TypeDef BspUSB_VCPMonitor = {
    .init_state =  BspUSB_None_Init,
};

/* internal function */
static void BspUSB_VCP_RecData_Callback(uint8_t *p_data, uint16_t len);
static void BspUSB_VCP_SendData_CPLT_Callback(uint8_t *p_data, uint32_t *size);

/* external function */
static BspUSB_Error_List BspUSB_VCP_Init(void);
static BspUSB_Error_List BspUSB_VCP_SendData(uint8_t *p_data, uint16_t len);
static void BspUSB_VCP_Set_Rx_Callback(BspUSB_Rx_Callback_Def callback);
static void BspUSB_VCP_Set_Tx_CPLT_Callback(BspUSB_Tx_Cplt_Callback_Def callback);
static BspUSB_VCP_TxStatistic_TypeDef BspUSB_VCP_Get_TxStatistic(void);

BspUSB_VCP_TypeDef BspUSB_VCP = {
    .init =  BspUSB_VCP_Init,
    .send =  BspUSB_VCP_SendData,
    .set_rx_callback = BspUSB_VCP_Set_Rx_Callback,
    .set_tx_cpl_callback = BspUSB_VCP_Set_Tx_CPLT_Callback,
    .get_tx_statistic = BspUSB_VCP_Get_TxStatistic,
};

static BspUSB_Error_List BspUSB_VCP_Init(void)
{
    if(BspUSB_VCPMonitor.init_state == BspUSB_None_Init)
    {
        /* send queue init_state */
        if(!Queue.create_auto(&BspUSB_VCPMonitor.SendQueue, "VCP Queue", USB_VCP_TX_BUFF_SIZE))
        {
            BspUSB_VCPMonitor.init_state = BspUSB_Error_QueueCreate;
            return BspUSB_Error_QueueCreate;
        }

        if(!USB_DEVICE_Init())
        {
            BspUSB_VCPMonitor.init_state = BspUSB_Error_Init;
            return BspUSB_Error_Init;
        }

        /* set callback */
        usb_setrec_callback(BspUSB_VCP_RecData_Callback);
        usb_settxcpl_callback(BspUSB_VCP_SendData_CPLT_Callback);
        
        memset(BspUSB_VCPMonitor.single_tx_buffer, 0, USB_VCP_MAX_TX_SIZE);
        BspUSB_VCPMonitor.init_state = BspUSB_Error_None;
        return BspUSB_Error_None;
    }

    return BspUSB_VCPMonitor.init_state;
}

static BspUSB_Error_List BspUSB_VCP_SendData(uint8_t *p_data, uint16_t len)
{
    uint16_t push_size = 0;
    uint16_t tx_size = 0;
    uint8_t *tx_src = NULL;
    uint8_t *push_src_addr = NULL;
    uint16_t cur_queue_size = 0;
    BspUSB_VCP_TxSource_List tx_src_type = BspUSB_VCP_TxSrc_Input;

    if((BspUSB_VCPMonitor.init_state == BspUSB_Error_None) && p_data && len)
    {
        if(BspUSB_VCPMonitor.tx_cnt == BspUSB_VCPMonitor.tx_fin_cnt)
        {
            /* check queue first */
            cur_queue_size = Queue.size(BspUSB_VCPMonitor.SendQueue); 
            if(cur_queue_size)
            {
                tx_src_type = BspUSB_VCP_TxSrc_Queue;

                if(cur_queue_size <= USB_VCP_MAX_TX_SIZE)
                {
                    tx_size = cur_queue_size;
                }
                else
                    /* queue size > USB_VCP_MAX_TX_SIZE */
                    tx_size = USB_VCP_MAX_TX_SIZE;

                Queue.pop(&BspUSB_VCPMonitor.SendQueue, BspUSB_VCPMonitor.single_tx_buffer, tx_size);
                tx_src = BspUSB_VCPMonitor.single_tx_buffer;
            
                /* after pop history data to current tx buff we need push current tx data to queue */
                if(Queue.remain(BspUSB_VCPMonitor.SendQueue) >= len)
                {
                    Queue.push(&BspUSB_VCPMonitor.SendQueue, p_data, len);
                }
                else
                    /* no mem space for incoming data */
                    BspUSB_VCPMonitor.tx_abort_cnt ++;
            }
            else
            {
                if(len > USB_VCP_MAX_TX_SIZE)
                {
                    push_size = len - USB_VCP_MAX_TX_SIZE;
                    push_src_addr = &p_data[USB_VCP_MAX_TX_SIZE];

                    tx_size = USB_VCP_MAX_TX_SIZE;
                }
                else
                    tx_size = len;

                tx_src = p_data;
            }

            if(CDC_Transmit_FS(tx_src, tx_size) != USBD_OK)
            {
                BspUSB_VCPMonitor.tx_err_cnt ++;

                /* mark current tx source if form queue we need to push data back to the queue head */
                if((tx_src_type == BspUSB_VCP_TxSrc_Queue) && (Queue.remain(BspUSB_VCPMonitor.SendQueue) >= tx_size))
                {
                    memset(BspUSB_VCPMonitor.single_tx_buffer, 0, USB_VCP_TX_BUFF_SIZE);
                    memcpy(BspUSB_VCPMonitor.single_tx_buffer, tx_src, tx_size);
                    cur_queue_size = Queue.size(BspUSB_VCPMonitor.SendQueue);
                    Queue.pop(&BspUSB_VCPMonitor.SendQueue, &BspUSB_VCPMonitor.single_tx_buffer[tx_size], cur_queue_size);

                    push_size = tx_size + cur_queue_size;
                    push_src_addr = BspUSB_VCPMonitor.single_tx_buffer;
                }
                else if(tx_src_type == BspUSB_VCP_TxSrc_Input)
                {
                    push_size = len;
                    push_src_addr = p_data;
                }
            }
            else
                BspUSB_VCPMonitor.tx_cnt ++;
        }
        else
        {
            push_size = len;
            push_src_addr = p_data;
        }

        if(push_size && push_src_addr)
        {
            /* push current send into queue for next time sending */
            if(Queue.remain(BspUSB_VCPMonitor.SendQueue) >= push_size)
            {
                Queue.push(&BspUSB_VCPMonitor.SendQueue, push_src_addr, push_size);
            }
            else
                BspUSB_VCPMonitor.tx_abort_cnt ++;
        }
    }
}

static BspUSB_VCP_TxStatistic_TypeDef BspUSB_VCP_Get_TxStatistic(void)
{
    BspUSB_VCP_TxStatistic_TypeDef statistic;

    statistic.tx_cnt = BspUSB_VCPMonitor.tx_cnt;
    statistic.tx_abort = BspUSB_VCPMonitor.tx_abort_cnt;
    statistic.tx_fin_cnt = BspUSB_VCPMonitor.tx_fin_cnt;
    statistic.tx_err_cnt = BspUSB_VCPMonitor.tx_err_cnt;

    return statistic;
}

static void BspUSB_VCP_Set_Rx_Callback(BspUSB_Rx_Callback_Def callback)
{
    BspUSB_VCPMonitor.rx_callback = callback;
}

static void BspUSB_VCP_Set_Tx_CPLT_Callback(BspUSB_Tx_Cplt_Callback_Def callback)
{
    BspUSB_VCPMonitor.tx_fin_callback = callback;
}

/* internel irq function */
static void BspUSB_VCP_RecData_Callback(uint8_t *p_data, uint16_t len)
{
    if((BspUSB_VCPMonitor.init_state ==  BspUSB_Error_None) && p_data && len)
    {
        BspUSB_VCPMonitor.rx_byte_sum += len;
        BspUSB_VCPMonitor.rx_irq_cnt ++;

        if(BspUSB_VCPMonitor.rx_callback)
            BspUSB_VCPMonitor.rx_callback(p_data, len);
    }
}

static void BspUSB_VCP_SendData_CPLT_Callback(uint8_t *p_data, uint32_t *size)
{
    if((BspUSB_VCPMonitor.init_state ==  BspUSB_Error_None) && p_data && size)
    {
        BspUSB_VCPMonitor.tx_fin_cnt ++;

        if(BspUSB_VCPMonitor.tx_fin_callback)
            BspUSB_VCPMonitor.tx_fin_callback(p_data, size);
    }
}





