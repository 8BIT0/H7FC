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
static bool Telemetry_RC_Sig_Init(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *receiver_obj);
static bool Telemetry_BindGimbalToChannel(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, uint16_t gimbal_tag, uint16_t min_range, uint16_t max_range);
static bool Telemetry_BindARMToChannel(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, uint16_t tag, uint16_t min_range, uint16_t max_range);
static bool Telemetry_BindDisARMToChannel(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, uint16_t tag, uint16_t min_range, uint16_t max_range);
static bool Telemetry_AddARMCombo(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, uint16_t tag, uint16_t min_range, uint16_t max_range);
static bool Telemetry_AddDisARMCombo(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, uint16_t tag, uint16_t min_range, uint16_t max_range);

void TaskTelemetry_Init(void)
{
    /* init receiver */
    if (Telemetry_RC_Sig_Init(&RC_Setting, &Receiver_Obj))
    {
        /* for crsf frame channel 1 is throttle */
        /* for sbus frame channel 3 is throttle */

        /* map function to channel */
        if (!Telemetry_BindGimbalToChannel(&RC_Setting, &Receiver_Obj.data.val_list[0], Telemetry_RC_Throttle, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MAX) ||
            !Telemetry_BindGimbalToChannel(&RC_Setting, &Receiver_Obj.data.val_list[1], Telemetry_RC_Pitch, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MAX) ||
            !Telemetry_BindGimbalToChannel(&RC_Setting, &Receiver_Obj.data.val_list[2], Telemetry_RC_Roll, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MAX) ||
            !Telemetry_BindGimbalToChannel(&RC_Setting, &Receiver_Obj.data.val_list[3], Telemetry_RC_Yaw, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MAX))
        {
            RC_Setting.init_state = false;
            RC_Setting.arm_state = TELEMETRY_SET_ARM;
        }
    }

    /* init radio */
}

void TaskTelemetry_Core(Task_Handle hdl)
{
    Telemetry_RC_Sig_Update(&RC_Setting, &Receiver_Obj);
}

/************************************** receiver ********************************************/
static bool Telemetry_RC_Sig_Init(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *receiver_obj)
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

static bool Telemetry_BindGimbalToChannel(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, uint16_t tag, uint16_t min_range, uint16_t max_range)
{
    Telemetry_ChannelSet_TypeDef *channel_set = NULL;

    if ((!RC_Input_obj) ||
        (!data_obj) ||
        (min_range < TELEMETRY_RC_CHANNEL_RANGE_MIN) ||
        (max_range > TELEMETRY_RC_CHANNEL_RANGE_MAX))
        return false;

    channel_set = (Telemetry_ChannelSet_TypeDef *)MMU_Malloc(sizeof(Telemetry_ChannelSet_TypeDef));

    if (!channel_set)
    {
        MMU_Free(channel_set);
        return false;
    }

    channel_set->channel_ptr = data_obj;
    channel_set->max = max_range;
    channel_set->min = min_range;

    switch (tag)
    {
    case Telemetry_RC_Throttle:
    case Telemetry_RC_Pitch:
    case Telemetry_RC_Roll:
    case Telemetry_RC_Yaw:
        RC_Input_obj->Gimbal[tag].combo_cnt = 1;
        List_ItemInit(&(RC_Input_obj->Gimbal[tag].combo_list), channel_set);
        break;

    default:
        return false;
    }
}

static bool Telemetry_BindARMToChannel(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, uint16_t tag, uint16_t min_range, uint16_t max_range)
{
    Telemetry_ChannelSet_TypeDef *channel_set = NULL;

    if ((!RC_Input_obj) ||
        (!data_obj) ||
        (min_range < TELEMETRY_RC_CHANNEL_RANGE_MIN) ||
        (max_range > TELEMETRY_RC_CHANNEL_RANGE_MAX))
        return false;

    channel_set = (Telemetry_ChannelSet_TypeDef *)MMU_Malloc(sizeof(Telemetry_ChannelSet_TypeDef));

    if (!channel_set)
    {
        MMU_Free(channel_set);
        return false;
    }

    channel_set->channel_ptr = data_obj;
    channel_set->max = max_range;
    channel_set->min = min_range;

    RC_Input_obj->ARM_Toggle.combo_cnt = 1;
    RC_Input_obj->ARM_Toggle.combo_list.mode = by_order;
    RC_Input_obj->ARM_Toggle.combo_list.compare_callback = NULL;
    List_ItemInit(&(RC_Input_obj->ARM_Toggle.combo_list), channel_set);

    return true;
}

static bool Telemetry_BindDisARMToChannel(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, uint16_t tag, uint16_t min_range, uint16_t max_range)
{
    Telemetry_ChannelSet_TypeDef *channel_set = NULL;

    if ((!RC_Input_obj) ||
        (!data_obj) ||
        (min_range < TELEMETRY_RC_CHANNEL_RANGE_MIN) ||
        (max_range > TELEMETRY_RC_CHANNEL_RANGE_MAX))
        return false;

    channel_set = (Telemetry_ChannelSet_TypeDef *)MMU_Malloc(sizeof(Telemetry_ChannelSet_TypeDef));

    if (!channel_set)
    {
        MMU_Free(channel_set);
        return false;
    }

    channel_set->channel_ptr = data_obj;
    channel_set->max = max_range;
    channel_set->min = min_range;

    RC_Input_obj->DisARM_Toggle.combo_cnt = 1;

    RC_Input_obj->DisARM_Toggle.combo_list.mode = by_order;
    RC_Input_obj->DisARM_Toggle.combo_list.compare_callback = NULL;
    List_ItemInit(&(RC_Input_obj->DisARM_Toggle.combo_list), channel_set);

    return true;
}

static bool Telemetry_AddARMCombo(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, uint16_t tag, uint16_t min_range, uint16_t max_range)
{
    Telemetry_ChannelSet_TypeDef *channel_set = NULL;
    item_obj *combo_item = NULL;

    if ((!RC_Input_obj) ||
        (!data_obj) ||
        (min_range < TELEMETRY_RC_CHANNEL_RANGE_MIN) ||
        (max_range > TELEMETRY_RC_CHANNEL_RANGE_MAX))
        return false;

    channel_set = (Telemetry_ChannelSet_TypeDef *)MMU_Malloc(sizeof(Telemetry_ChannelSet_TypeDef));

    if (!channel_set)
    {
        MMU_Free(channel_set);
        return false;
    }

    combo_item = (item_obj *)MMU_Malloc(sizeof(item_obj));

    if (!combo_item)
    {
        MMU_Free(combo_item);
        return false;
    }

    channel_set->channel_ptr = data_obj;
    channel_set->max = max_range;
    channel_set->min = min_range;
    RC_Input_obj->ARM_Toggle.combo_cnt ++;

    List_ItemInit(combo_item, channel_set);
    List_Insert_Item(&(RC_Input_obj->ARM_Toggle.combo_list), combo_item);

    return true;
}

static bool Telemetry_AddDisARMCombo(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, uint16_t tag, uint16_t min_range, uint16_t max_range)
{
    Telemetry_ChannelSet_TypeDef *channel_set = NULL;
    item_obj *combo_item = NULL;

    if ((!RC_Input_obj) ||
        (!data_obj) ||
        (min_range < TELEMETRY_RC_CHANNEL_RANGE_MIN) ||
        (max_range > TELEMETRY_RC_CHANNEL_RANGE_MAX))
        return false;

    channel_set = (Telemetry_ChannelSet_TypeDef *)MMU_Malloc(sizeof(Telemetry_ChannelSet_TypeDef));

    if (!channel_set)
    {
        MMU_Free(channel_set);
        return false;
    }

    combo_item = (item_obj *)MMU_Malloc(sizeof(item_obj));

    if (!combo_item)
    {
        MMU_Free(combo_item);
        return false;
    }

    channel_set->channel_ptr = data_obj;
    channel_set->max = max_range;
    channel_set->min = min_range;
    RC_Input_obj->DisARM_Toggle.combo_cnt ++;

    List_ItemInit(combo_item, channel_set);
    List_Insert_Item(&(RC_Input_obj->ARM_Toggle.combo_list), combo_item);

    return true;
}

static bool Telemetry_Check_ARMSig_Input(Telemetry_RCInput_TypeDef *RC_Input_obj)
{
    if ((!RC_Input_obj) || (!RC_Input_obj->init_state))
        return false;
}

static void Telemetry_RC_Sig_Update(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *receiver_obj)
{
    if ((!RC_Input_obj) || (!receiver_obj) || (!RC_Input_obj->init_state))
        return;

    SrvReceiver.get(receiver_obj);
}
