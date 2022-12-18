#include "Task_Telemetry.h"
#include "DataPipe.h"
#include "Srv_Receiver.h"
#include "system_cfg.h"
#include "IO_Definition.h"
#include "runtime.h"
#include "mmu.h"

/* internal variable */
static SrvReceiverObj_TypeDef Receiver_Obj;
static Telemetry_RCInput_TypeDef RC_Setting;

/* internal funciotn */
static void Telemetry_RC_Sig_Update(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *receiver_obj);
static void Telemetry_RC_Sig_Init(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *receiver_obj);
static bool Telemetry_MapToChannel(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *data_obj, uint16_t gimbal_tag, uint16_t min_range, uint16_t max_range);

void TaskTelemetry_Init(void)
{
    /* init receiver */
    Telemetry_RC_Sig_Init(&RC_Setting, &Receiver_Obj);

    /* init radio */
}

void TaskTelemetry_Core(Task_Handle hdl)
{
    Telemetry_RC_Sig_Update(&RC_Setting, &Receiver_Obj);
}

/************************************** receiver ********************************************/
static void Telemetry_RC_Sig_Init(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *receiver_obj)
{
    uint8_t *port_ptr = NULL;

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

    return RC_Input_obj->init_state;
}

static bool Telemetry_MapToChannel(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *data_obj, uint16_t tag, uint16_t min_range, uint16_t max_range)
{
    uint8_t index = 0;
    bool MapGimbal = false;
    Telemetry_ChannelSet_TypeDef *channel_set = NULL;

    if ((!RC_Input_obj) ||
        (!data_obj) ||
        (min_range < TELEMETRY_RC_CHANNEL_MIN_VAL) ||
        (max_range > TELEMETRY_RC_CHANNEL_MAX_VAL))
        return false;

    switch (tag)
    {
    case Telemetry_RC_Pitch:
        index = 0;
        MapGimbal = true;
        break;

    case Telemetry_RC_Roll:
        index = 1;
        MapGimbal = true;
        break;

    case Telemetry_RC_Throttle:
        index = 2;
        MapGimbal = true;
        break;

    case Telemetry_RC_Yaw:
        index = 3;
        MapGimbal = true;
        break;

    default:
        return false;
    }

    if (MapGimbal)
    {
        RC_Input_obj->Gimbal[index].combo_cnt = 1;
        channel_set = (Telemetry_ChannelSet_TypeDef *)MMU_Malloc(sizeof(Telemetry_ChannelSet_TypeDef));

        if (!channel_set)
        {
            MMU_Free(channel_set);
            return false;
        }

        memset(channel_set, NULL, sizeof(Telemetry_ChannelSet_TypeDef));

        channel_set->channel_ptr = data_obj;
        channel_set->max = max_range;
        channel_set->min = min_range;

        List_ItemInit(&RC_Input_obj->Gimbal[index].combo_list, channel_set);
    }
}

static bool Telemetry_Check_ARMSig_Input(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *receiver_obj)
{
    if ((!RC_Input_obj) || (!receiver_obj) || (!RC_Input_obj->init_state))
        return;
}

static void Telemetry_RC_Sig_Update(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *receiver_obj)
{
    if ((!RC_Input_obj) || (!receiver_obj) || (!RC_Input_obj->init_state))
        return;

    SrvReceiver.get(receiver_obj);
}
