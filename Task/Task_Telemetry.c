/* code: 8_B!T0
 * Telemetry Task we use this task to get and process data from RC receiver and radio
 * OSD Tune Gimbal code : throttle down / Pitch down / Yaw right max / Roll lift max / keep this position for 5S
 */
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
static DataPipeObj_TypeDef Receiver_Smp_DataPipe;

/* internal funciotn */
static void Telemetry_RC_Sig_Update(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *receiver_obj);
static bool Telemetry_RC_Sig_Init(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *receiver_obj);
static bool Telemetry_BindGimbalToChannel(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, uint16_t gimbal_tag, uint16_t min_range, uint16_t max_range);
static bool Telemetry_BindToggleToChannel(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, Telemetry_RCFuncMap_TypeDef *toggle, uint16_t min_range, uint16_t max_range);
static bool Telemetry_AddToggleCombo(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, Telemetry_RCFuncMap_TypeDef *toggle, uint16_t min_range, uint16_t max_range);

void TaskTelemetry_Init(void)
{
    /* init receiver */
    if (Telemetry_RC_Sig_Init(&RC_Setting, &Receiver_Obj))
    {
        /* for crsf frame channel 1 is throttle */
        /* for sbus frame channel 3 is throttle */

        /* bind to channel */
        if (!Telemetry_BindGimbalToChannel(&RC_Setting, &Receiver_Obj.data.val_list[0], Telemetry_RC_Throttle, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MAX) ||
            !Telemetry_BindGimbalToChannel(&RC_Setting, &Receiver_Obj.data.val_list[1], Telemetry_RC_Pitch, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MAX) ||
            !Telemetry_BindGimbalToChannel(&RC_Setting, &Receiver_Obj.data.val_list[2], Telemetry_RC_Roll, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MAX) ||
            !Telemetry_BindGimbalToChannel(&RC_Setting, &Receiver_Obj.data.val_list[3], Telemetry_RC_Yaw, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MAX) ||
            !Telemetry_BindToggleToChannel(&RC_Setting, &Receiver_Obj.data.val_list[4], &RC_Setting.ARM_Toggle, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MID) || /* bind arm & disarm to channel */
            !Telemetry_BindToggleToChannel(&RC_Setting, &Receiver_Obj.data.val_list[5], &RC_Setting.Buzzer_Toggle, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MID) || /* bind buzzer to channel */
            !Telemetry_BindToggleToChannel(&RC_Setting, &Receiver_Obj.data.val_list[6], &RC_Setting.ControlMode_Toggle, ) || /* bind control mode toggle */
            !Telemetry_AddToggleCombo(&RC_Setting, &Receiver_Obj.data.val_list[6], &RC_Setting.ControlMode_Toggle, ) ||
            !Telemetry_AddToggleCombo(&RC_Setting, &Receiver_Obj.data.val_list[6], &RC_Setting.ControlMode_Toggle, ) ||
            !Telemetry_BindToggleToChannel(&RC_Setting, &Receiver_Obj.data.val_list[], &RC_Setting.OSD_Toggle, ) || /* bind osd tune to channel */
            !Telemetry_AddToggleCombo(&RC_Setting, &Receiver_Obj.data.val_list[], &RC_Setting.OSD_Toggle, ) ||
            !Telemetry_AddToggleCombo(&RC_Setting, &Receiver_Obj.data.val_list[], &RC_Setting.OSD_Toggle, ) ||
            !Telemetry_AddToggleCombo(&RC_Setting, &Receiver_Obj.data.val_list[], &RC_Setting.OSD_Toggle, ))
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

/************************************** telemetry receiver section ********************************************/
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
    RC_Input_obj->buzz_state = false;
    RC_Input_obj->osd_tune_state = false;
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

static bool Telemetry_BindToggleToChannel(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, Telemetry_RCFuncMap_TypeDef *toggle, uint16_t min_range, uint16_t max_range)
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

    toggle->combo_cnt = 1;
    toggle->combo_list.mode = by_order;
    toggle->combo_list.compare_callback = NULL;
    List_ItemInit(&(toggle->combo_list), channel_set);

    return true;
}

static bool Telemetry_AddToggleCombo(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, Telemetry_RCFuncMap_TypeDef *toggle, uint16_t min_range, uint16_t max_range)
{
    Telemetry_ChannelSet_TypeDef *channel_set = NULL;
    item_obj *item = NULL;

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

    item = (item_obj *)MMU_Malloc(sizeof(item_obj));

    if (!item)
    {
        MMU_Free(item);
        return false;
    }

    toggle->combo_cnt++;
    List_ItemInit(item, channel_set);
    List_Insert_Item(&toggle->combo_list, item);

    return true;
}

static Telemetry_ToggleData_TypeDef Telemetry_Toggle_Check(Telemetry_RCFuncMap_TypeDef *toggle)
{
    Telemetry_ToggleData_TypeDef toggle_val;

    toggle_val.pos = 0;
    toggle_val.state = false;
    item_obj *nxt = &toggle->combo_list;
    Telemetry_ChannelSet_TypeDef *channel_data = NULL;

    if (!toggle)
        return toggle_val;

    /* do not use list traverse here */
    while (nxt)
    {
        channel_data = (Telemetry_ChannelSet_TypeDef *)nxt->data;

        if (!channel_data ||
            ((*(uint16_t *)channel_data->channel_ptr) > channel_data->max) ||
            ((*(uint16_t *)channel_data->channel_ptr) < channel_data->min))
            break;

        nxt = nxt->nxt;
        toggle_val.pos++;
    }

    if (toggle_val.pos == toggle->combo_cnt)
        toggle_val.state = true;

    return toggle_val;
}

static uint16_t Telemetry_Check_Gimbal(Telemetry_RCFuncMap_TypeDef *gimbal)
{
    Telemetry_ChannelSet_TypeDef *gimbal_channel = NULL;

    if(!gimbal)
        return TELEMETRY_RC_CHANNEL_RANGE_MIN;

    gimbal_channel = gimbal->combo_list.data;

    if(!gimbal_channel)
        return TELEMETRY_RC_CHANNEL_RANGE_MIN;
    
    if(*gimbal_channel->channel_ptr < gimbal_channel->min)
        return gimbal_channel->min;

    if(*gimbal_channel->channel_ptr > gimbal_channel->max)
        return gimbal_channel->max;

    return *gimbal_channel->channel_ptr;
}

static void Telemetry_RC_Sig_Update(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *receiver_obj)
{
    SrvReceiverData_TypeDef receiver_data;

    memset(&receiver_data, 0, sizeof(receiver_data));
    if ((!RC_Input_obj) || (!receiver_obj) || (!RC_Input_obj->init_state))
        return;

    receiver_data = SrvReceiver.get(receiver_obj);

    /* notic disarm and osd tune can not enable at the same time */
    /* when power on and arm toggle on remote is set on disarm we force it to arm */

    if(!receiver_data.failsafe)
    {
        if (!RC_Input_obj->osd_tune_state)
        {
            /* check arm & disarm */
            RC_Input_obj->arm_state = Telemetry_Toggle_Check(&RC_Input_obj->ARM_Toggle).state;

            /* check control mode */
            RC_Input_obj->control_mode = Telemetry_Toggle_Check(&RC_Input_obj->ControlMode_Toggle).pos;

            /* check control mode inedx range */
            if((RC_Input_obj->control_mode > Telemetry_Control_Mode_AUTO) || (RC_Input_obj->control_mode < Telemetry_Control_Mode_ACRO))
                RC_Input_obj->control_mode = Telemetry_Control_Mode_Default;
        }

        /* get gimbal channel */
        for(uint8_t i = Telemetry_RC_Throttle; i < Telemetry_Gimbal_TagSum; i++)
            RC_Input_obj->gimbal_val[i] = Telemetry_Check_Gimbal(&RC_Input_obj->Gimbal[i]);

        /* check buzzer toggle */
        RC_Input_obj->buzz_state = Telemetry_Toggle_Check(&RC_Input_obj->Buzzer_Toggle).state;

        if (RC_Input_obj->arm_state)
        {
            /* check osd tune toggle */
            RC_Input_obj->osd_tune_state = Telemetry_Toggle_Check(&RC_Input_obj->OSD_Toggle).state;
        }
    }
    else
    {
        RC_Input_obj->arm_state = TELEMETRY_SET_ARM;
    
        for(uint8_t i = Telemetry_RC_Throttle; i < Telemetry_Gimbal_TagSum; i++)
            RC_Input_obj->gimbal_val[i] = TELEMETRY_RC_CHANNEL_RANGE_MIN;

    }

}




/************************************** telemetry radio section ********************************************/

