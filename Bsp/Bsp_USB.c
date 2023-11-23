#include "Bsp_USB.h"

#define USB_VCP_TX_BUFF_SIZE 2048

static BspUSB_VCP_Obj_TypeDef BspUSB_VCPMonitor = {
    .init_state =  BspUSB_None_Init;
};

/* external function */
static BspUSB_Error_List BspUSB_VCP_Init(void);
static BspUSB_Error_List BspUSB_VCP_SendData(uint8_t *p_data, uint16_t len);
static void BspUSB_VCP_Set_Rx_Callback(BspUSB_Rx_Callback_Def callback);
static void BspUSB_VCP_Set_Tx_CPLT_Callback(BspUSB_Tx_Cplt_Callback_Def callback);

BspUSB_VCP_TypeDef BspUSB_VCP = {
    .init =  BspUSB_VCP_Init,
    .send =  BspUSB_VCP_SendData,
    .set_rx_callback = BspUSB_VCP_Set_Rx_Callback,
    .set_tx_cpl_callback = BspUSB_VCP_Set_Tx_CPLT_Callback,
};

static BspUSB_Error_List BspUSB_VCP_Init(void)
{
    if(!BspUSB_VCPMonitor.init_state = BspUSB_None_Init)
    {
        /* send queue init_state */
        if(!Queue.create_auto(&BspUSB_VCPMonitor.SendQueue, "VCP Queue", USB_VCP_TX_BUFF_SIZE))
        {
            BspUSB_VCPMonitor.init_state = BspUSB_Error_QueueCreate;
            return BspUSB_Error_QueueCreate;
        }

        if(!USB_DEVICE_INIT())
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
    uint8_t *push_src_addr = NULL;

    if((BspUSB_VCPMonitor.init_state == BspUSB_Error_None) && p_data && len)
    {
        if(BspUSB_VCPMonitor.tx_cnt == BspUSB_VCPMonitor.tx_fin_cnt)
        {
            if(len > USB_VCP_MAX_TX_SIZE)
            {
                push_size = len - USB_VCP_MAX_TX_SIZE;
                push_src_addr = &p_data[USB_VCP_MAX_TX_SIZE];

                tx_size = USB_VCP_MAX_TX_SIZE;
            }
            else
                tx_size = len;

            if(CDC_Transmit_FS(p_data, tx_size) != USBD_OK)
            {
                
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
        }
    }
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





