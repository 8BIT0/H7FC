#include "Task_Telemetry.h"
#include "Srv_Receiver.h"
#include "system_cfg.h"
#include "IO_Definition.h"

static SrvReceiverObj_TypeDef Receiver_Obj;
bool Receiver_Init_State = false;

void TaskTelemetry_Init(void)
{
    uint8_t port_ptr = NULL; 

    memset(&Receiver_Obj, 0, sizeof(Receiver_Obj));

    Receiver_Obj.port_type = Receiver_Port_Serial;
    Receiver_Obj.Frame_type = Receiver_Type_CRSF;
    
    /* create receiver port */
    switch(Receiver_Obj.port_type)
    {
        case Receiver_Port_Serial:
            port_ptr = SrvReceiver.create_serial_obj(RECEIVER_PORT, 
                                                     RECEIVER_RX_DMA, 
                                                     RECEIVER_RX_DMA_STREAM,
                                                     RECEIVER_TX_DMA,
                                                     RECEIVER_TX_DMA_STREAM,
                                                     false,
                                                     Uart4_TxPin,
                                                     Uart4_RxPin);
            
            if(Receiver_Obj.Frame_type == Receiver_Type_Sbus)
            {
                /* set inverter pin */
            }
        break;

        case Receiver_Port_Spi:
            /* still in developing */
            port_ptr = SrvReceiver.create_spi_obj();
        break;

        default:
            return false;
    }   

    /* init receiver object */
    Receiver_Init_State = SrvReceiver.init(&Receiver_Obj, port_ptr);

    /* init radio port & radio object */
}

void TaskTelemetry_Core(Task_Handle hdl)
{
}

static void TaskTelemetry_Check_Receiver(void)
{

}
