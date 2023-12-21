/* code: 8_B!T0
 * Telemetry Task we use this task to get and process data from RC receiver and radio
 * OSD Tune Gimbal code : throttle down / Pitch down / Yaw right max / Roll right max / keep this position for 5S
 *
 * deflaut receiver list
 * channel 1  roll
 * channel 2  pitch
 * channel 3  throttle
 * channel 4  yaw
 * channel 5  arm toggle            two   pos : on/off
 * channel 6  buzzer toggle         two   pos : on/off
 * channel 7  control mode switch   three pos : pos 1/2/3
 * channel 8  taking over           auto reset toggle hold for enable
 * channel 9  flip over             two   pos : on/off
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
#include "Srv_OsCommon.h"
#include "util.h"
#include "Srv_ComProto.h"

#define CRSF_TX_PIN Uart4_TxPin
#define CRSF_RX_PIN Uart4_RxPin

#define SBUS_TX_PIN Uart4_TxPin
#define SBUS_RX_PIN Uart4_RxPin

static SrvReceiverObj_TypeDef Receiver_Obj;
static Telemetry_RCInput_TypeDef RC_Setting;
static Telemetry_Monitor_TypeDef Telemetry_Monitor;
static bool RCData_To_Configuretor = false;
static uint32_t TaskTelemetry_Period = 0;
DataPipe_CreateDataObj(Telemetry_RCSig_TypeDef, Rc);

/* internal funciotn */
static Telemetry_RCSig_TypeDef Telemetry_RC_Sig_Update(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *receiver_obj);
static bool Telemetry_RC_Sig_Init(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *receiver_obj);
static bool Telemetry_BindGimbalToChannel(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, uint16_t gimbal_tag, uint16_t min_range, uint16_t mid_val, uint16_t max_range);
static bool Telemetry_BindToggleToChannel(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, Telemetry_RCFuncMap_TypeDef *toggle, uint16_t trigger_min_range, uint16_t trigger_max_range);
static bool Telemetry_AddToggleCombo(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, Telemetry_RCFuncMap_TypeDef *toggle, uint16_t trigger_min_range, uint16_t trigger_max_range);
static void Telemetry_Enable_GimbalDeadZone(Telemetry_RCFuncMap_TypeDef *gimbal, uint16_t scope);
static uint16_t Telemetry_SplitScopeValue_Into(uint8_t pcs);
static bool Telemetry_Bind_Gimbal(uint8_t throttle_ch, uint8_t pitch_ch, uint8_t roll_ch, uint8_t yaw_ch);
static bool Telemetry_Bind_Toggle(uint8_t arm_toggle_ch, uint8_t mode_toggle_ch, uint8_t buzzer_toggle_ch, uint8_t flipover_toggle_ch, uint8_t takingover_toggle_ch);
static void Telemetry_ConvertRCData_To_ControlData(Telemetry_RCSig_TypeDef RCSig, ControlData_TypeDef *CTLSig);
static bool Telemetry_Bind_OSDCombo(void);
static bool Telemetry_Bind_CalibCombo(void);

void TaskTelemetry_Set_DataIO_Enable(bool state)
{
    RCData_To_Configuretor = state;
}

void TaskTelemetry_Init(uint32_t period)
{
    memset(&Telemetry_Monitor, 0, sizeof(Telemetry_Monitor));

    Telemetry_Monitor.Init_Rt = SrvOsCommon.get_os_ms();

    /* init receiver */
    if (Telemetry_RC_Sig_Init(&RC_Setting, &Receiver_Obj))
    {
        /* for crsf frame channel 1 is throttle */
        /* for sbus frame channel 3 is throttle */
        if (Receiver_Obj.Frame_type == Receiver_Type_CRSF)
        {
            /* set crsf receiver map */
            /* bind to channel */
            if (Telemetry_Bind_Gimbal(Channel_3, Channel_2, Channel_1, Channel_4) && \
                Telemetry_Bind_Toggle(Channel_5, Channel_7, Channel_6, Channel_9, Channel_8) && \
                Telemetry_Bind_OSDCombo() && \
                Telemetry_Bind_CalibCombo())
            {
                RC_Setting.sig.arm_state = TELEMETRY_SET_ARM;

                /* set gimbal center dead zone */
                Telemetry_Enable_GimbalDeadZone(&RC_Setting.Gimbal[Gimbal_Pitch], 50);
                Telemetry_Enable_GimbalDeadZone(&RC_Setting.Gimbal[Gimbal_Roll], 50);
                Telemetry_Enable_GimbalDeadZone(&RC_Setting.Gimbal[Gimbal_Yaw], 50);

                /* set datapipe */
                memset(&Receiver_Smp_DataPipe, 0, sizeof(Receiver_Smp_DataPipe));
                memset(DataPipe_DataObjAddr(Rc), 0, sizeof(Telemetry_RCSig_TypeDef));

                Receiver_Smp_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Rc);
                Receiver_Smp_DataPipe.data_size = DataPipe_DataSize(Rc);

                DataPipe_Enable(&Receiver_Smp_DataPipe);
            }
        }
        else if (Receiver_Obj.Frame_type == Receiver_Type_Sbus)
        {
            /* still in developing */
        }

        Telemetry_Monitor.lst_arm_state = TELEMETRY_SET_ARM;
    }

    TaskTelemetry_Period = period;
}

void Telemetry_blink(void)
{
    uint32_t Rt = 0;
    static uint32_t Lst_Rt = 0;
    static bool led_state = false;

    // DebugPin.ctl(Debug_PB4, true);
    // DebugPin.ctl(Debug_PB4, false);

    Rt = SrvOsCommon.get_os_ms();

    if ((Rt % 100 == 0) && (Lst_Rt != Rt))
    {
        led_state = !led_state;
        Lst_Rt = Rt;
    }

    DevLED.ctl(Led1, led_state);
    // DevLED.ctl(Led3, led_state);
}

static bool Telemetry_Led_Control(bool state)
{
    DevLED.ctl(Led1, state);
}

void TaskTelemetry_Core(void const *arg)
{
    uint32_t sys_time = SrvOsCommon.get_os_ms();
    ControlData_TypeDef RC_CtlData;
    
    while(1)
    {
        memset(&RC_CtlData, 0, sizeof(ControlData_TypeDef)); 
        
        // Telemetry_blink();
        
        /* RC receiver process */
        Telemetry_ConvertRCData_To_ControlData(Telemetry_RC_Sig_Update(&RC_Setting, &Receiver_Obj), &RC_CtlData);

        /* pipe data out */
        DataPipe_SendTo(&Receiver_Smp_DataPipe, &Receiver_hub_DataPipe);
        
        SrvOsCommon.precise_delay(&sys_time, TaskTelemetry_Period);
    }
}

/************************************** telemetry receiver section ********************************************/
static bool Telemetry_RC_Sig_Init(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *receiver_obj)
{
    uint8_t *port_ptr = NULL;

    memset(receiver_obj, 0, sizeof(receiver_obj));

    receiver_obj->port_type = Receiver_Port_Serial;
    receiver_obj->Frame_type = Receiver_Type_CRSF;
    receiver_obj->OSDTune_TriggerMs = 0;

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
            /* still in developing */
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

    RC_Input_obj->init_state = SrvReceiver.get_scope(receiver_obj, \
                                                     &Telemetry_Monitor.receiver_value_max, \
                                                     &Telemetry_Monitor.receiver_value_mid, \
                                                     &Telemetry_Monitor.receiver_value_min);

    return RC_Input_obj->init_state;
}

static bool Telemetry_BindGimbalToChannel(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, uint16_t tag, uint16_t min_range, uint16_t mid_val, uint16_t max_range)
{
    Telemetry_ChannelSet_TypeDef *channel_set = NULL;

    if ((!RC_Input_obj) ||
        (!data_obj) ||
        (min_range < Telemetry_Monitor.receiver_value_min) ||
        (max_range > Telemetry_Monitor.receiver_value_max))
        return false;

    channel_set = (Telemetry_ChannelSet_TypeDef *)SrvOsCommon.malloc(sizeof(Telemetry_ChannelSet_TypeDef));

    if (!channel_set)
    {
        SrvOsCommon.free(channel_set);
        return false;
    }

    channel_set->channel_ptr = data_obj;
    channel_set->max = max_range;
    channel_set->min = min_range;
    channel_set->center_deadzone_scope = 0;
    channel_set->enable_deadzone = false;

    switch (tag)
    {
        case Gimbal_Throttle:
        case Gimbal_Pitch:
        case Gimbal_Roll:
        case Gimbal_Yaw:
            RC_Input_obj->Gimbal[tag].combo_cnt = 1;
            List_ItemInit(&(RC_Input_obj->Gimbal[tag].combo_list), channel_set);
            return true;

        default:
            return false;
    }
}

static bool Telemetry_BindToggleToChannel(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, Telemetry_RCFuncMap_TypeDef *toggle, uint16_t trigger_min_range, uint16_t trigger_max_range)
{
    Telemetry_ChannelSet_TypeDef *channel_set = NULL;

    if ((!RC_Input_obj) ||
        (!data_obj) ||
        (trigger_min_range < Telemetry_Monitor.receiver_value_min) ||
        (trigger_max_range > Telemetry_Monitor.receiver_value_max))
        return false;

    channel_set = (Telemetry_ChannelSet_TypeDef *)SrvOsCommon.malloc(sizeof(Telemetry_ChannelSet_TypeDef));

    if (!channel_set)
    {
        SrvOsCommon.free(channel_set);
        return false;
    }

    channel_set->channel_ptr = data_obj;
    channel_set->max = trigger_max_range;
    channel_set->min = trigger_min_range;

    toggle->combo_cnt = 1;
    toggle->combo_list.mode = by_order;
    toggle->combo_list.compare_callback = NULL;
    List_ItemInit(&(toggle->combo_list), channel_set);

    return true;
}

static bool Telemetry_AddToggleCombo(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, Telemetry_RCFuncMap_TypeDef *toggle, uint16_t trigger_min_range, uint16_t trigger_max_range)
{
    Telemetry_ChannelSet_TypeDef *channel_set = NULL;
    item_obj *item = NULL;

    if ((!RC_Input_obj) ||
        (!data_obj) ||
        (trigger_min_range < Telemetry_Monitor.receiver_value_min) ||
        (trigger_max_range > Telemetry_Monitor.receiver_value_max))
        return false;

    channel_set = (Telemetry_ChannelSet_TypeDef *)SrvOsCommon.malloc(sizeof(Telemetry_ChannelSet_TypeDef));

    if (!channel_set)
    {
        SrvOsCommon.free(channel_set);
        return false;
    }

    item = (item_obj *)SrvOsCommon.malloc(sizeof(item_obj));

    if (!item)
    {
        SrvOsCommon.free(item);
        return false;
    }

    channel_set->channel_ptr = data_obj;
    channel_set->max = trigger_max_range;
    channel_set->min = trigger_min_range;

    toggle->combo_cnt++;
    List_ItemInit(item, channel_set);
    List_Insert_Item(&toggle->combo_list, item);

    return true;
}

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
            pos_reg |= 1 << toggle_val.pos;
            toggle_val.cnt++;
        }

        toggle_val.pos++;
        nxt = nxt->nxt;
    }

    /* get the number of the bit on set */
    if (Get_OnSet_Bit_Num(pos_reg) == 1)
    {
        /* only on bit been set */
        toggle_val.pos = Get_Bit_Index(pos_reg);
    }
    else
    {
        /* many bits been set on pos_reg */
        toggle_val.pos = 0;
    }

    if (toggle_val.cnt == toggle->combo_cnt)
        toggle_val.state = true;

    return toggle_val;
}

static void Telemetry_Enable_GimbalDeadZone(Telemetry_RCFuncMap_TypeDef *gimbal, uint16_t scope)
{
    Telemetry_ChannelSet_TypeDef *gimbal_channel = NULL;

    if (gimbal)
    {
        gimbal_channel = gimbal->combo_list.data;

        if (gimbal_channel)
        {
            gimbal_channel->center_deadzone_scope = scope;
            gimbal_channel->enable_deadzone = true;
        }
    }
}

static uint16_t Telemetry_Check_Gimbal(Telemetry_RCFuncMap_TypeDef *gimbal)
{
    Telemetry_ChannelSet_TypeDef *gimbal_channel = NULL;

    if (gimbal == NULL)
        return TELEMETRY_RC_CHANNEL_RANGE_MIN;

    gimbal_channel = gimbal->combo_list.data;

    if ((gimbal_channel == NULL) || (gimbal_channel->min >= gimbal_channel->max))
        return TELEMETRY_RC_CHANNEL_RANGE_MIN;

    if (*gimbal_channel->channel_ptr < gimbal_channel->min)
        return gimbal_channel->min;

    if (*gimbal_channel->channel_ptr > gimbal_channel->max)
        return gimbal_channel->max;

    return *gimbal_channel->channel_ptr;
}

static uint16_t Telemetry_GimbalToPercent(Telemetry_RCFuncMap_TypeDef *gimbal)
{
    Telemetry_ChannelSet_TypeDef *gimbal_channel = NULL;
    uint16_t gimbal_range = 0;
    float percent = 0.0f;

    if (gimbal == NULL)
        return 0;

    gimbal_channel = gimbal->combo_list.data;

    if ((gimbal_channel == NULL) || (gimbal_channel->min >= gimbal_channel->max))
        return 0;

    if (gimbal_channel->enable_deadzone)
    {
        if (*gimbal_channel->channel_ptr < (gimbal_channel->mid + gimbal_channel->center_deadzone_scope) ||
            (*gimbal_channel->channel_ptr > (gimbal_channel->mid - gimbal_channel->center_deadzone_scope)))
            return 50;
    }

    gimbal_range = gimbal_channel->max - gimbal_channel->min;
    percent = (float)(*gimbal_channel->channel_ptr - gimbal_channel->min) / gimbal_range;
    percent *= 100;

    return (uint16_t)percent;
}

static Telemetry_RCSig_TypeDef Telemetry_RC_Sig_Update(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *receiver_obj)
{
    SrvReceiverData_TypeDef receiver_data;
    Telemetry_RCSig_TypeDef sig_tmp;

    memset(&receiver_data, 0, sizeof(receiver_data));
    memset(&sig_tmp, 0, sizeof(sig_tmp));

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
        RC_Input_obj->sig.time_stamp = RC_Input_obj->update_rt;

        RC_Input_obj->sig.channel_sum = Receiver_Obj.channel_num;

        /* get gimbal channel */
        for (uint8_t i = 0; i < receiver_obj->channel_num; i++)
        {
            if (i < Gimbal_Sum)
            {
                Telemetry_Check_Gimbal(&RC_Input_obj->Gimbal[i]);
                RC_Input_obj->sig.gimbal_percent[i] = Telemetry_GimbalToPercent(&RC_Input_obj->Gimbal[i]);
            }

            RC_Input_obj->sig.channel[i] = receiver_data.val_list[i];
        }

        /* check taking over toggle */
        RC_Input_obj->sig.taking_over = Telemetry_Toggle_Check(&RC_Input_obj->TakingOver_Toggle).state;

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
        else
        {
            RC_Input_obj->sig.arm_state = DRONE_ARM;
            RC_Input_obj->sig.cali_state = false;
            RC_Input_obj->sig.flip_over = false;
            RC_Input_obj->sig.taking_over = false;
        }

        if (RC_Input_obj->sig.arm_state)
        {
            /* check osd tune toggle */
            if (Telemetry_Toggle_Check(&RC_Input_obj->OSD_Toggle).state)
            {
                if (Receiver_Obj.OSDTune_TriggerMs == 0)
                    Receiver_Obj.OSDTune_TriggerMs = SrvOsCommon.get_os_ms();

                if (SrvOsCommon.get_os_ms() - Receiver_Obj.OSDTune_TriggerMs >= TELEMETRY_OSDTUNE_POSHOLD)
                    RC_Input_obj->sig.osd_tune_state = true;
            }
            else
            {
                Receiver_Obj.OSDTune_TriggerMs = 0;
                
                /* check calibrate */
                RC_Input_obj->sig.cali_state = Telemetry_Toggle_Check(&RC_Input_obj->CLB_Toggle).state;
            
                /* check flip over toggle */
                RC_Input_obj->sig.flip_over = Telemetry_Toggle_Check(&RC_Input_obj->FlipOver_Toggle).state;
            }
        }
        else
        {
            RC_Input_obj->sig.osd_tune_state = false;
            RC_Input_obj->sig.cali_state = false;
            RC_Input_obj->sig.flip_over = false;
        }

        RC_Input_obj->update_rt = receiver_data.time_stamp;
        RC_Input_obj->sig.update_interval = RC_Input_obj->update_rt - RC_Input_obj->lst_update_rt;

        RC_Input_obj->lst_update_rt = RC_Input_obj->update_rt;
        RC_Input_obj->sig.failsafe = false;

        /* is recover form failsafe */
        if (Telemetry_Monitor.recover_failsafe)
        {
            /* if current arm state equal to before fall into failsafe's
             * then sig is fine or else the arm signal has been change from recovered */
            if ((Telemetry_Monitor.lst_arm_state == TELEMETRY_SET_ARM) && (RC_Input_obj->sig.arm_state == TELEMETRY_SET_DISARM))
            {
                RC_Input_obj->sig.failsafe = true;
                return;
            }

            Telemetry_Monitor.recover_failsafe = false;
        }

        /* if toggle switch into disarm but throttle currently is upper then 5% percent input
         * force throttle value to 0
         * only when physical throttle gimbal actually down to lowest so we update lst_arm_state */
        if ((RC_Input_obj->sig.arm_state == TELEMETRY_SET_DISARM) && (Telemetry_Monitor.lst_arm_state == TELEMETRY_SET_ARM))
        {
            if (RC_Input_obj->sig.gimbal_percent[0] >= TELEMETRY_RC_THROTTLE_PERCENT_ALERT)
            {
                RC_Input_obj->sig.gimbal_percent[0] = 0;
                RC_Input_obj->sig.arm_state = TELEMETRY_SET_ARM;
            }
        }

        Telemetry_Monitor.lst_arm_state = RC_Input_obj->sig.arm_state;

        Telemetry_Led_Control(false);
    }
    else
    {
        RC_Input_obj->sig.failsafe = true;
        // RC_Input_obj->sig.arm_state = TELEMETRY_SET_ARM;
        RC_Input_obj->sig.osd_tune_state = false;
        RC_Input_obj->sig.buzz_state = false;
        // RC_Input_obj->sig.control_mode = Telemetry_Control_Mode_Default;

        for (uint8_t i = Gimbal_Throttle; i < Gimbal_Sum; i++)
            RC_Input_obj->sig.gimbal_percent[i] = 0;

        Telemetry_Monitor.recover_failsafe = true;
        Telemetry_Led_Control(true);
    }

    memcpy(&sig_tmp, &RC_Input_obj->sig, sizeof(Telemetry_RCSig_TypeDef));

    return sig_tmp;
}

static uint16_t Telemetry_SplitScopeValue_Into(uint8_t pcs)
{
    uint16_t range = Telemetry_Monitor.receiver_value_max - Telemetry_Monitor.receiver_value_min;

    if(pcs)
        return range / pcs;

    return 0;
}

static bool Telemetry_Bind_Gimbal(uint8_t throttle_ch, uint8_t pitch_ch, uint8_t roll_ch, uint8_t yaw_ch)
{
    if((throttle_ch == pitch_ch) || \
       (throttle_ch == roll_ch) || \
       (throttle_ch == yaw_ch) || \
       (pitch_ch == roll_ch) || \
       (pitch_ch == yaw_ch) || \
       (roll_ch == yaw_ch))
        return false;

    if( !Telemetry_BindGimbalToChannel(&RC_Setting, \
                                       &Receiver_Obj.data.val_list[throttle_ch], \
                                       Gimbal_Throttle, \
                                       Telemetry_Monitor.receiver_value_min, \
                                       Telemetry_Monitor.receiver_value_mid, \
                                       Telemetry_Monitor.receiver_value_max) ||
        !Telemetry_BindGimbalToChannel(&RC_Setting, \
                                       &Receiver_Obj.data.val_list[pitch_ch], \
                                       Gimbal_Pitch, \
                                       Telemetry_Monitor.receiver_value_min, \
                                       Telemetry_Monitor.receiver_value_mid, \
                                       Telemetry_Monitor.receiver_value_max) ||
        !Telemetry_BindGimbalToChannel(&RC_Setting, \
                                       &Receiver_Obj.data.val_list[roll_ch], \
                                       Gimbal_Roll, \
                                       Telemetry_Monitor.receiver_value_min, \
                                       Telemetry_Monitor.receiver_value_mid, \
                                       Telemetry_Monitor.receiver_value_max) ||
        !Telemetry_BindGimbalToChannel(&RC_Setting, \
                                       &Receiver_Obj.data.val_list[yaw_ch], \
                                       Gimbal_Yaw, \
                                       Telemetry_Monitor.receiver_value_min, \
                                       Telemetry_Monitor.receiver_value_mid, \
                                       Telemetry_Monitor.receiver_value_max))
        return false;

    return true;
}

static bool Telemetry_Bind_Toggle(uint8_t arm_toggle_ch, uint8_t mode_toggle_ch, uint8_t buzzer_toggle_ch, uint8_t flipover_toggle_ch, uint8_t takingover_toggle_ch)
{
    int16_t start = 0;
    int16_t end = 0;
    
    /* bind arm toggle */
    if(!Telemetry_BindToggleToChannel(&RC_Setting, \
                                      &Receiver_Obj.data.val_list[arm_toggle_ch], \
                                      &RC_Setting.ARM_Toggle, \
                                      Telemetry_Monitor.receiver_value_min, \
                                      Telemetry_Monitor.receiver_value_mid))
        return false;

    /* bind buzzer toggle */
    if(!Telemetry_BindToggleToChannel(&RC_Setting, \
                                      &Receiver_Obj.data.val_list[buzzer_toggle_ch], \
                                      &RC_Setting.Buzzer_Toggle, \
                                      Telemetry_Monitor.receiver_value_min, \
                                      Telemetry_Monitor.receiver_value_mid))
        return false;
    
    /* bind control mode switcher toggle */ 
    uint16_t split_val = Telemetry_SplitScopeValue_Into(3);
    start = Telemetry_Monitor.receiver_value_min;
    end = Telemetry_Monitor.receiver_value_min + split_val;
    if(!Telemetry_BindToggleToChannel(&RC_Setting, \
                                      &Receiver_Obj.data.val_list[mode_toggle_ch], \
                                      &RC_Setting.ControlMode_Toggle, \
                                      start, \
                                      end))
        return false;

    start = end;
    end += split_val;
    if(!Telemetry_AddToggleCombo(&RC_Setting, \
                                 &Receiver_Obj.data.val_list[mode_toggle_ch], \
                                 &RC_Setting.ControlMode_Toggle, \
                                 start, \
                                 end))
        return false;

    start = end;
    end = Telemetry_Monitor.receiver_value_max;
    if(!Telemetry_AddToggleCombo(&RC_Setting, \
                                 &Receiver_Obj.data.val_list[mode_toggle_ch], \
                                 &RC_Setting.ControlMode_Toggle, \
                                 start, \
                                 end))
       return false;

    /* bind control taking over toggle (toggle must can auto reset) */
    if(!Telemetry_BindToggleToChannel(&RC_Setting, \
                                      &Receiver_Obj.data.val_list[takingover_toggle_ch], \
                                      &RC_Setting.TakingOver_Toggle, \
                                      Telemetry_Monitor.receiver_value_mid, \
                                      Telemetry_Monitor.receiver_value_max))
        return false;

    if(!Telemetry_BindToggleToChannel(&RC_Setting, \
                                      &Receiver_Obj.data.val_list[flipover_toggle_ch], \
                                      &RC_Setting.FlipOver_Toggle, \
                                      Telemetry_Monitor.receiver_value_mid, \
                                      Telemetry_Monitor.receiver_value_max))
        return false;

    return true;
}

/* const gimbal position */
static bool Telemetry_Bind_OSDCombo(void)
{
    int16_t start = Telemetry_Monitor.receiver_value_min;
    int16_t end = start + 100;

    if(!Telemetry_AddToggleCombo(&RC_Setting, &Receiver_Obj.data.val_list[1], &RC_Setting.OSD_Toggle, start, end))
        return false;

    start = Telemetry_Monitor.receiver_value_max - 100;
    end = Telemetry_Monitor.receiver_value_max;
    if(!Telemetry_AddToggleCombo(&RC_Setting, &Receiver_Obj.data.val_list[3], &RC_Setting.OSD_Toggle, start, end))
        return false;
    
    start = Telemetry_Monitor.receiver_value_min;
    end = start + 100;
    if(!Telemetry_AddToggleCombo(&RC_Setting, &Receiver_Obj.data.val_list[0], &RC_Setting.OSD_Toggle, start, end))
        return false;

    return true;
}

/* const gimbal position */
static bool Telemetry_Bind_CalibCombo(void)
{
    int16_t start = Telemetry_Monitor.receiver_value_min;
    int16_t end = start + 100;

    if(!Telemetry_AddToggleCombo(&RC_Setting, &Receiver_Obj.data.val_list[2], &RC_Setting.CLB_Toggle, start, end) || \
       !Telemetry_AddToggleCombo(&RC_Setting, &Receiver_Obj.data.val_list[1], &RC_Setting.CLB_Toggle, start, end) || \
       !Telemetry_AddToggleCombo(&RC_Setting, &Receiver_Obj.data.val_list[0], &RC_Setting.CLB_Toggle, start, end) || \
       !Telemetry_AddToggleCombo(&RC_Setting, &Receiver_Obj.data.val_list[3], &RC_Setting.CLB_Toggle, start, end))
       return false;

    return true;
}

static void Telemetry_ConvertRCData_To_ControlData(Telemetry_RCSig_TypeDef RCSig, ControlData_TypeDef *CTLSig)
{
    if(CTLSig)
    {
        CTLSig->update_time_stamp = RCSig.time_stamp;
    }
}
