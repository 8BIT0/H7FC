#include "Task_Telemetry.h"
#include "DataPipe.h"
#include "Srv_Receiver.h"
#include "system_cfg.h"
#include "IO_Definition.h"
#include "runtime.h"

/* internal variable */
static SrvReceiverObj_TypeDef Receiver_Obj;
static Telemetry_RCInput_TypeDef RC_Setting;

/* internal funciotn */
static void TaskTelemetry_RC_Sig_Update(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *receiver_obj);
static void TaskTelemetry_RC_Sig_Init(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *receiver_obj);

void TaskTelemetry_Init(void)
{
    /* init receiver */
    TaskTelemetry_RC_Sig_Init(&RC_Setting, &Receiver_Obj);

    /* init radio */
}

void TaskTelemetry_Core(Task_Handle hdl)
{
    TaskTelemetry_RC_Sig_Update(&RC_Setting, &Receiver_Obj);
}

/************************************** receiver ********************************************/
static void TaskTelemetry_RC_Sig_Init(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *receiver_obj)
{
    uint8_t port_ptr = NULL;

    memset(receiver_obj, 0, sizeof(receiver_obj));

    receiver_obj->port_type = Receiver_Port_Serial;
    receiver_obj->Frame_type = Receiver_Type_CRSF;

    /* create receiver port */
    switch (receiver_obj->port_type)
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

        if (receiver_obj->Frame_type == Receiver_Type_Sbus)
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
    RC_Input_obj->init_state = SrvReceiver.init(receiver_obj, port_ptr);

    RC_Input_obj->arm_state = TELEMETRY_SET_ARM;
    RC_Input_obj->control_mode = Telemetry_Control_Mode_Default;
    RC_Input_obj->module_enable = TELEMETRY_DISABLE_ALL_MODULE;
}

static bool TaskTelemetry_Check_ARMSig_Input(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *receiver_obj)
{
    if ((!RC_Input_obj) || (!receiver_obj) || (!RC_Input_obj->init_state))
        return;
}

static void TaskTelemetry_RC_Sig_Update(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *receiver_obj)
{
    if ((!RC_Input_obj) || (!receiver_obj) || (!RC_Input_obj->init_state))
        return;

    SrvReceiver.get(receiver_obj);
}
