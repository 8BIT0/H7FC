#include "Srv_Flow.h"
#include "Dev_Flow_3901U.h"
#include "Bsp_Uart.h"
#include "HW_Def.h"

#define SrvFlow_TaskPeriod_Base 50  /* 50Hz */

/* external function */
static bool SrvFlow_Init(SrvFlowObj_TypeDef *obj);
static bool SrvFlow_Get_Data(SrvFlowObj_TypeDef *flow_obj, SrvFlowData_TypeDef *p_data);

SrvFlow_TypeDef SrvFlow = {
    .init =  SrvFlow_Init,
    .get_data = SrvFlow_Get_Data,
};

static bool SrvFlow_Init(SrvFlowObj_TypeDef *flow_obj)
{
    void *p_obj = NULL;
    void *port_obj = NULL;

    if(flow_obj && (flow_obj->type != Flow_None))
    {
        switch((uint8_t)flow_obj->type)
        {
            case Flow_3901U:
                /* use uart6 connect with flow sensor */
                p_obj = SrvOsCommon.malloc(DevFlow3901U_Size);
                port_obj = SrvOsCommon.malloc(BspUartObj_Size);
                if ((p_obj == NULL) || (port_obj == NULL))
                    return false;

                To_DevFlow3901U_Obj(p_obj)->get_sys_time = SrvOsCommon.get_os_ms;
                flow_obj->obj = p_obj;
                flow_obj->api = &DevFlow3901;

                if (DevFlow3901.init(To_DevFlow3901U_Obj(p_obj)) == 0)
                    return false;
                
                /* port init */
                To_BspUart_Obj(port_obj)->baudrate = FLOW_3901U_BAUDRATE;
                To_BspUart_Obj(port_obj)->cust_data_addr = p_obj;
                To_BspUart_Obj(port_obj)->irq_type = BspUart_IRQ_Type_Idle;
                To_BspUart_Obj(port_obj)->instance = FLOW_PORT;
                To_BspUart_Obj(port_obj)->rx_io = Uart6_RxPin;
                To_BspUart_Obj(port_obj)->tx_io = Uart6_TxPin;
                To_BspUart_Obj(port_obj)->pin_swap = false;
                To_BspUart_Obj(port_obj)->rx_dma = FLOW_RX_DMA;
                To_BspUart_Obj(port_obj)->rx_stream = FLOW_RX_DMA_STREAM;
                To_BspUart_Obj(port_obj)->tx_dma = FLOW_TX_DMA;
                To_BspUart_Obj(port_obj)->tx_stream = FLOW_TX_DMA_STREAM;
                To_BspUart_Obj(port_obj)->rx_size = 128;
                To_BspUart_Obj(port_obj)->rx_buf = SrvOsCommon.malloc(To_BspUart_Obj(port_obj)->rx_size);
                if (To_BspUart_Obj(port_obj)->rx_buf == NULL)
                    return false;

                BspUart.init(port_obj);
                /* set port parse callback */
                BspUart.set_rx_callback(To_BspUart_Obj(port_obj), (BspUART_Callback *)DevFlow3901.recv);
                return true;

            default: return false;
        }

        if (flow_obj->thread_freq)
        {
            /* create sample task */
            /* reserved */
        }
    }

    return false;
}

static bool SrvFlow_Get_Data(SrvFlowObj_TypeDef *flow_obj, SrvFlowData_TypeDef *p_data)
{
    if (flow_obj && p_data)
    {
        switch ((uint8_t)flow_obj->type)
        {
            case Flow_3901U:
                return true;

            default: return false;
        }
    }

    return false;
}

