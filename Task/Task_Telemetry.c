/* code: 8_B!T0
 * Telemetry Task we use this task to get and process data from RC receiver and radio
 * OSD Tune Gimbal code : throttle down / Pitch down / Yaw right max / Roll right max / keep this position for 5S
 *
 * deflaut receiver list
 * channel 1  pitch
 * channel 2  roll
 * channel 3  throttle
 * channel 4  yaw
 * channel 5  aux1
 * channel 6  aux2
 * channel 7  aux3
 * channel 8  aux4
 * channel 9  aux5
 * channel 10 aux6
 * channel 11 aux7
 * channel 12 aux8
 * channel 13 aux9
 * channel 14 aux10
 * channel 15 aux11
 * channel 16 aux12
 *
 */
#include "Task_Telemetry.h"
#include "DataPipe.h"
#include "Srv_Receiver.h"
#include "system_cfg.h"
#include "IO_Definition.h"
#include "runtime.h"
#include "mmu.h"
#include "util.h"

#define CRSF_TX_PIN Uart4_TxPin
#define CRSF_RX_PIN Uart4_RxPin

#define SBUS_TX_PIN Uart4_TxPin
#define SBUS_RX_PIN Uart4_RxPin

/* internal variable */
static SrvReceiverObj_TypeDef Receiver_Obj;
static Telemetry_RCInput_TypeDef RC_Setting;
static bool RCData_To_Configuretor = false;
DataPipe_CreateDataObj(Telemetry_RCSig_TypeDef, Rc);

/* internal funciotn */
static Telemetry_RCSig_TypeDef Telemetry_RC_Sig_Update(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *receiver_obj);
static bool Telemetry_RC_Sig_Init(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *receiver_obj);
static bool Telemetry_BindGimbalToChannel(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, uint16_t gimbal_tag, uint16_t min_range, uint16_t max_range);
static bool Telemetry_BindToggleToChannel(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, Telemetry_RCFuncMap_TypeDef *toggle, uint16_t min_range, uint16_t max_range);
static bool Telemetry_AddToggleCombo(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, Telemetry_RCFuncMap_TypeDef *toggle, uint16_t min_range, uint16_t max_range);

void TaskTelemetry_Set_DataIO_Enable(bool state)
{
    RCData_To_Configuretor = state;
}

void TaskTelemetry_Init(void)
{
    /* init receiver */
    if (Telemetry_RC_Sig_Init(&RC_Setting, &Receiver_Obj))
    {
        /* for crsf frame channel 1 is throttle */
        /* for sbus frame channel 3 is throttle */
        if (Receiver_Obj.Frame_type == Receiver_Type_CRSF)
        {
            /* set crsf receiver map */
            /* bind to channel */
            if (!Telemetry_BindGimbalToChannel(&RC_Setting, &Receiver_Obj.data.val_list[2], Telemetry_RC_Throttle, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MAX) ||
                !Telemetry_BindGimbalToChannel(&RC_Setting, &Receiver_Obj.data.val_list[1], Telemetry_RC_Pitch, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MAX) ||
                !Telemetry_BindGimbalToChannel(&RC_Setting, &Receiver_Obj.data.val_list[0], Telemetry_RC_Roll, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MAX) ||
                !Telemetry_BindGimbalToChannel(&RC_Setting, &Receiver_Obj.data.val_list[3], Telemetry_RC_Yaw, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MAX) ||
                !Telemetry_BindToggleToChannel(&RC_Setting, &Receiver_Obj.data.val_list[4], &RC_Setting.ARM_Toggle, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MID) ||    /* bind arm & disarm to channel */
                !Telemetry_BindToggleToChannel(&RC_Setting, &Receiver_Obj.data.val_list[5], &RC_Setting.Buzzer_Toggle, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MID) || /* bind buzzer to channel */
                !Telemetry_BindToggleToChannel(&RC_Setting, &Receiver_Obj.data.val_list[6], &RC_Setting.ControlMode_Toggle, TELEMETRY_RC_CHANNEL_RANGE_MIN, 300) ||                       /* bind control mode toggle */
                !Telemetry_AddToggleCombo(&RC_Setting, &Receiver_Obj.data.val_list[6], &RC_Setting.ControlMode_Toggle, 900, 1100) ||
                !Telemetry_AddToggleCombo(&RC_Setting, &Receiver_Obj.data.val_list[6], &RC_Setting.ControlMode_Toggle, 1650, TELEMETRY_RC_CHANNEL_RANGE_MAX) ||
                !Telemetry_BindToggleToChannel(&RC_Setting, &Receiver_Obj.data.val_list[2], &RC_Setting.OSD_Toggle, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MIN + 100) || /* bind osd tune to channel */
                !Telemetry_AddToggleCombo(&RC_Setting, &Receiver_Obj.data.val_list[1], &RC_Setting.OSD_Toggle, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MIN + 100) ||
                !Telemetry_AddToggleCombo(&RC_Setting, &Receiver_Obj.data.val_list[3], &RC_Setting.OSD_Toggle, TELEMETRY_RC_CHANNEL_RANGE_MAX - 100, TELEMETRY_RC_CHANNEL_RANGE_MAX) ||
                !Telemetry_AddToggleCombo(&RC_Setting, &Receiver_Obj.data.val_list[0], &RC_Setting.OSD_Toggle, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MIN + 100))
            {
                RC_Setting.init_state = false;
                RC_Setting.sig.arm_state = TELEMETRY_SET_ARM;

                /* set datapipe */
                memset(&Receiver_Smp_DataPipe, 0, sizeof(Receiver_Smp_DataPipe));
                memset(DataPipe_DataObjAddr(Rc), 0, sizeof(Telemetry_RCSig_TypeDef));

                Receiver_Smp_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Rc);
                Receiver_Smp_DataPipe.data_size = sizeof(Telemetry_RCSig_TypeDef);
            }
        }
        else if (Receiver_Obj.Frame_type == Receiver_Type_Sbus)
        {
        }
    }

    /* init radio */
}

void Telemetry_blink(void)
{
    SYSTEM_RunTime Rt = 0;
    static SYSTEM_RunTime Lst_Rt = 0;
    static bool led_state = false;

    // DebugPin.ctl(Debug_PB4, true);
    // DebugPin.ctl(Debug_PB4, false);

    Rt = Get_CurrentRunningMs();

    if ((Rt % 100 == 0) && (Lst_Rt != Rt))
    {
        led_state = !led_state;
        Lst_Rt = Rt;
    }

    DevLED.ctl(Led1, led_state);
    // DevLED.ctl(Led3, led_state);
}

void TaskTelemetry_Core(Task_Handle hdl)
{
    Telemetry_blink();

    DataPipe_DataObj(Rc) = Telemetry_RC_Sig_Update(&RC_Setting, &Receiver_Obj);

    /* pipe data out */
    DataPipe_SendTo(&Receiver_Smp_DataPipe, &Receiver_Ctl_DataPipe);
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
        if (receiver_obj->Frame_type == Receiver_Type_CRSF)
        {
            port_ptr = SrvReceiver.create_serial_obj(RECEIVER_PORT,
                                                     RECEIVER_CRSF_RX_DMA,
                                                     RECEIVER_CRSF_RX_DMA_STREAM,
                                                     RECEIVER_CRSF_TX_DMA,
                                                     RECEIVER_CRSF_TX_DMA_STREAM,
                                                     false,
                                                     CRSF_TX_PIN,
                                                     CRSF_RX_PIN);
        }
        else if (receiver_obj->Frame_type == Receiver_Type_Sbus)
        {
            port_ptr = SrvReceiver.create_serial_obj(RECEIVER_PORT,
                                                     RECEIVER_SBUS_RX_DMA,
                                                     RECEIVER_SBUS_RX_DMA_STREAM,
                                                     RECEIVER_SBUS_TX_DMA,
                                                     RECEIVER_SBUS_TX_DMA_STREAM,
                                                     false,
                                                     SBUS_TX_PIN,
                                                     SBUS_RX_PIN);

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

    RC_Input_obj->sig.arm_state = TELEMETRY_SET_ARM;
    RC_Input_obj->sig.buzz_state = false;
    RC_Input_obj->sig.osd_tune_state = false;
    RC_Input_obj->sig.control_mode = Telemetry_Control_Mode_Default;
    RC_Input_obj->sig.module_enable = TELEMETRY_DISABLE_ALL_MODULE;

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
        return true;

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

    channel_set->channel_ptr = data_obj;
    channel_set->max = max_range;
    channel_set->min = min_range;

    toggle->combo_cnt++;
    List_ItemInit(item, channel_set);
    List_Insert_Item(&toggle->combo_list, item);

    return true;
}

/* still bug */
static Telemetry_ToggleData_TypeDef Telemetry_Toggle_Check(Telemetry_RCFuncMap_TypeDef *toggle)
{
    Telemetry_ToggleData_TypeDef toggle_val;
    uint8_t pos_reg = 0;

    toggle_val.pos = 0;
    toggle_val.cnt = 0;
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
            (((*(uint16_t *)channel_data->channel_ptr) < channel_data->max) &&
             ((*(uint16_t *)channel_data->channel_ptr) > channel_data->min)))
        {
            pos_reg |= 1 << toggle_val.cnt;
            toggle_val.cnt++;
        }

        nxt = nxt->nxt;
    }

    /* get the number of the bit on set */
    if(Get_OnSet_Bit_Num(pos_reg) == 1)
    {
        /* only on bit been set */
        toggle_val.pos = Get_Bit_Index(pos_reg);
    }
    else
        toggle_val.pos = 0;

    if (toggle_val.cnt == toggle->combo_cnt)
        toggle_val.state = true;

    return toggle_val;
}

static uint16_t Telemetry_Check_Gimbal(Telemetry_RCFuncMap_TypeDef *gimbal)
{
    Telemetry_ChannelSet_TypeDef *gimbal_channel = NULL;

    if (!gimbal)
        return TELEMETRY_RC_CHANNEL_RANGE_MIN;

    gimbal_channel = gimbal->combo_list.data;

    if (!gimbal_channel)
        return TELEMETRY_RC_CHANNEL_RANGE_MIN;

    if (*gimbal_channel->channel_ptr < gimbal_channel->min)
        return gimbal_channel->min;

    if (*gimbal_channel->channel_ptr > gimbal_channel->max)
        return gimbal_channel->max;

    return *gimbal_channel->channel_ptr;
}

static Telemetry_RCSig_TypeDef Telemetry_RC_Sig_Update(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *receiver_obj)
{
    SrvReceiverData_TypeDef receiver_data;
    Telemetry_RCSig_TypeDef sig_tmp;

    memset(&receiver_data, 0, sizeof(receiver_data));
    memset(&sig_tmp, 0, sizeof(sig_tmp));

    sig_tmp.gimbal_val[Telemetry_RC_Throttle] = TELEMETRY_RC_CHANNEL_RANGE_MIN;
    sig_tmp.gimbal_val[Telemetry_RC_Pitch] = TELEMETRY_RC_CHANNEL_RANGE_MIN;
    sig_tmp.gimbal_val[Telemetry_RC_Roll] = TELEMETRY_RC_CHANNEL_RANGE_MIN;
    sig_tmp.gimbal_val[Telemetry_RC_Yaw] = TELEMETRY_RC_CHANNEL_RANGE_MIN;

    if ((!RC_Input_obj) || (!receiver_obj) || (!RC_Input_obj->init_state))
        return sig_tmp;

    receiver_data = SrvReceiver.get(receiver_obj);
    RC_Input_obj->rssi = 0;
    RC_Input_obj->link_quality = 0;

    /* notic disarm and osd tune can not enable at the same time */
    /* when power on and arm toggle on remote is set on disarm we force it to arm */

    if (!receiver_data.failsafe)
    {
        RC_Input_obj->rssi = receiver_data.rssi;
        RC_Input_obj->link_quality = receiver_data.link_quality;

        /* get gimbal channel */
        for (uint8_t i = Telemetry_RC_Throttle; i < Telemetry_Gimbal_TagSum; i++)
            RC_Input_obj->sig.gimbal_val[i] = Telemetry_Check_Gimbal(&RC_Input_obj->Gimbal[i]);

        /* check buzzer toggle */
        RC_Input_obj->sig.buzz_state = Telemetry_Toggle_Check(&RC_Input_obj->Buzzer_Toggle).state;

        if (!RC_Input_obj->sig.osd_tune_state)
        {
            /* check arm & disarm */
            RC_Input_obj->sig.arm_state = Telemetry_Toggle_Check(&RC_Input_obj->ARM_Toggle).state;

            /* check control mode */
            RC_Input_obj->sig.control_mode = Telemetry_Toggle_Check(&RC_Input_obj->ControlMode_Toggle).pos;

            /* check control mode inedx range */
            if ((RC_Input_obj->sig.control_mode > Telemetry_Control_Mode_AUTO) || (RC_Input_obj->sig.control_mode < Telemetry_Control_Mode_ACRO))
                RC_Input_obj->sig.control_mode = Telemetry_Control_Mode_Default;
        }

        if (RC_Input_obj->sig.arm_state)
        {
            /* check osd tune toggle */
            RC_Input_obj->sig.osd_tune_state = Telemetry_Toggle_Check(&RC_Input_obj->OSD_Toggle).state;
        }
    }
    else
    {
        RC_Input_obj->sig.arm_state = TELEMETRY_SET_ARM;
        RC_Input_obj->sig.osd_tune_state = false;
        RC_Input_obj->sig.buzz_state = false;
        RC_Input_obj->sig.control_mode = Telemetry_Control_Mode_Default;

        for (uint8_t i = Telemetry_RC_Throttle; i < Telemetry_Gimbal_TagSum; i++)
            RC_Input_obj->sig.gimbal_val[i] = TELEMETRY_RC_CHANNEL_RANGE_MIN;
    }

    memcpy(&sig_tmp, &RC_Input_obj->sig, sizeof(Telemetry_RCSig_TypeDef));

    return sig_tmp;
}

/************************************** telemetry radio section ********************************************/
