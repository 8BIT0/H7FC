#include "Bsp_USB.h"

#define USB_VCP_TX_BUFF_SIZE 2048

static BspUSB_VCP_Obj_TypeDef BspUSB_VCPMonitor = {
    .init_state =  BspUSB_None_Init;
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
        
        BspUSB_VCPMonitor.init_state = BspUSB_Error_None;
        return BspUSB_Error_None;
    }

    return BspUSB_VCPMonitor.init_state;
}

static BspUSB_Error_List BspUSB_VCP_SendData(uint8_t *p_data, uint16_t len)
{
    bool to_queue = false;

    if((BspUSB_VCPMonitor.init_state ==  BspUSB_Error_None) && p_data && len)
    {
        if(BspUSB_VCPMonitor.tx_cnt == BspUSB_VCPMonitor.tx_fin_cnt)
        {
            
        }
        else
            to_queue = true;
        
        if(to_queue)
        {
            /* push current send into queue for next time sending */
        }
    }
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





