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
#include "IO_Definition.h"
#include "Srv_OsCommon.h"
#include "util.h"
#include "Srv_ComProto.h"

#define CRSF_TX_PIN Uart4_TxPin
#define CRSF_RX_PIN Uart4_RxPin

#define SBUS_TX_PIN Uart4_TxPin
#define SBUS_RX_PIN Uart4_RxPin

/* internal variable */
/* MAVLink message List */
SrvComProto_MsgInfo_TypeDef TaskProto_MAV_RawIMU;
SrvComProto_MsgInfo_TypeDef TaskProto_MAV_ScaledIMU;
SrvComProto_MsgInfo_TypeDef TaskProto_MAV_RcChannel;
SrvComProto_MsgInfo_TypeDef TaskProto_MAV_MotoChannel;
SrvComProto_MsgInfo_TypeDef TaskProto_MAV_Attitude;

static SrvReceiverObj_TypeDef Receiver_Obj;
static Telemetry_RCInput_TypeDef RC_Setting;
static Telemetry_Monitor_TypeDef Telemetry_Monitor;
static bool RCData_To_Configuretor = false;
static bool Telemetry_MavProto_Enable = false;
static uint32_t TaskTelemetry_Period = 0;
static Telemetry_PortMonitor_TypeDef PortMonitor = {.init = false};
DataPipe_CreateDataObj(Telemetry_RCSig_TypeDef, Rc);

/* internal funciotn */
static Telemetry_RCSig_TypeDef Telemetry_RC_Sig_Update(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *receiver_obj);
static bool Telemetry_RC_Sig_Init(Telemetry_RCInput_TypeDef *RC_Input_obj, SrvReceiverObj_TypeDef *receiver_obj);
static bool Telemetry_BindGimbalToChannel(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, uint16_t gimbal_tag, uint16_t min_range, uint16_t mid_val, uint16_t max_range);
static bool Telemetry_BindToggleToChannel(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, Telemetry_RCFuncMap_TypeDef *toggle, uint16_t min_range, uint16_t max_range);
static bool Telemetry_AddToggleCombo(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, Telemetry_RCFuncMap_TypeDef *toggle, uint16_t min_range, uint16_t max_range);
static void Telemetry_Enable_GimbalDeadZone(Telemetry_RCFuncMap_TypeDef *gimbal, uint16_t scope);

/* redio section */

/* default vcp port section */
static void Telemetry_DefaultPort_Init(Telemetry_PortMonitor_TypeDef *monitor);
static void Telemetry_DefaultPort_TxCplt_Callback(uint8_t *p_data, uint32_t *size);
static bool Telemetry_RadioPort_Init(void);
static bool Telemetry_MAV_Msg_Init(void);
static void Telemetry_Port_Rx_Callback(uint32_t RecObj_addr, uint8_t *p_data, uint16_t size);

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
            if (Telemetry_BindGimbalToChannel(&RC_Setting, &Receiver_Obj.data.val_list[2], Telemetry_RC_Throttle, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MID, TELEMETRY_RC_CHANNEL_RANGE_MAX) &&
                Telemetry_BindGimbalToChannel(&RC_Setting, &Receiver_Obj.data.val_list[1], Telemetry_RC_Pitch, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MID, TELEMETRY_RC_CHANNEL_RANGE_MAX) &&
                Telemetry_BindGimbalToChannel(&RC_Setting, &Receiver_Obj.data.val_list[0], Telemetry_RC_Roll, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MID, TELEMETRY_RC_CHANNEL_RANGE_MAX) &&
                Telemetry_BindGimbalToChannel(&RC_Setting, &Receiver_Obj.data.val_list[3], Telemetry_RC_Yaw, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MID, TELEMETRY_RC_CHANNEL_RANGE_MAX) &&
                Telemetry_BindToggleToChannel(&RC_Setting, &Receiver_Obj.data.val_list[4], &RC_Setting.ARM_Toggle, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MID) &&    /* bind arm & disarm to channel */
                Telemetry_BindToggleToChannel(&RC_Setting, &Receiver_Obj.data.val_list[5], &RC_Setting.Buzzer_Toggle, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MID) && /* bind buzzer to channel */
                Telemetry_BindToggleToChannel(&RC_Setting, &Receiver_Obj.data.val_list[6], &RC_Setting.ControlMode_Toggle, TELEMETRY_RC_CHANNEL_RANGE_MIN, 300) &&                       /* bind control mode toggle */
                Telemetry_AddToggleCombo(&RC_Setting, &Receiver_Obj.data.val_list[6], &RC_Setting.ControlMode_Toggle, 900, 1100) &&
                Telemetry_AddToggleCombo(&RC_Setting, &Receiver_Obj.data.val_list[6], &RC_Setting.ControlMode_Toggle, 1650, TELEMETRY_RC_CHANNEL_RANGE_MAX) &&
                Telemetry_BindToggleToChannel(&RC_Setting, &Receiver_Obj.data.val_list[2], &RC_Setting.OSD_Toggle, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MIN + 100) && /* bind osd tune to channel */
                Telemetry_AddToggleCombo(&RC_Setting, &Receiver_Obj.data.val_list[1], &RC_Setting.OSD_Toggle, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MIN + 100) &&
                Telemetry_AddToggleCombo(&RC_Setting, &Receiver_Obj.data.val_list[3], &RC_Setting.OSD_Toggle, TELEMETRY_RC_CHANNEL_RANGE_MAX - 100, TELEMETRY_RC_CHANNEL_RANGE_MAX) &&
                Telemetry_AddToggleCombo(&RC_Setting, &Receiver_Obj.data.val_list[0], &RC_Setting.OSD_Toggle, TELEMETRY_RC_CHANNEL_RANGE_MIN, TELEMETRY_RC_CHANNEL_RANGE_MIN + 100))
            {
                RC_Setting.sig.arm_state = TELEMETRY_SET_ARM;

                /* set gimbal center dead zone */
                Telemetry_Enable_GimbalDeadZone(&RC_Setting.Gimbal[Telemetry_RC_Pitch], 50);
                Telemetry_Enable_GimbalDeadZone(&RC_Setting.Gimbal[Telemetry_RC_Roll], 50);
                Telemetry_Enable_GimbalDeadZone(&RC_Setting.Gimbal[Telemetry_RC_Yaw], 50);

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
        }

        Telemetry_Monitor.lst_arm_state = TELEMETRY_SET_ARM;
    }

    /* init radio port */
    Telemetry_RadioPort_Init();

    /* init radio protocol*/
    Telemetry_MavProto_Enable = Telemetry_MAV_Msg_Init();

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

    while(1)
    {
        // Telemetry_blink();

        DataPipe_DataObj(Rc) = Telemetry_RC_Sig_Update(&RC_Setting, &Receiver_Obj);

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

    return RC_Input_obj->init_state;
}

static bool Telemetry_BindGimbalToChannel(Telemetry_RCInput_TypeDef *RC_Input_obj, uint16_t *data_obj, uint16_t tag, uint16_t min_range, uint16_t mid_val, uint16_t max_range)
{
    Telemetry_ChannelSet_TypeDef *channel_set = NULL;

    if ((!RC_Input_obj) ||
        (!data_obj) ||
        (min_range < TELEMETRY_RC_CHANNEL_RANGE_MIN) ||
        (max_range > TELEMETRY_RC_CHANNEL_RANGE_MAX))
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

    channel_set = (Telemetry_ChannelSet_TypeDef *)SrvOsCommon.malloc(sizeof(Telemetry_ChannelSet_TypeDef));

    if (!channel_set)
    {
        SrvOsCommon.free(channel_set);
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
    channel_set->max = max_range;
    channel_set->min = min_range;

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

    sig_tmp.gimbal_percent[Telemetry_RC_Throttle] = 0;
    sig_tmp.gimbal_percent[Telemetry_RC_Pitch] = 0;
    sig_tmp.gimbal_percent[Telemetry_RC_Roll] = 0;
    sig_tmp.gimbal_percent[Telemetry_RC_Yaw] = 0;

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
            if (i < Telemetry_Gimbal_TagSum)
            {
                Telemetry_Check_Gimbal(&RC_Input_obj->Gimbal[i]);
                RC_Input_obj->sig.gimbal_percent[i] = Telemetry_GimbalToPercent(&RC_Input_obj->Gimbal[i]);
            }

            RC_Input_obj->sig.channel[i] = receiver_data.val_list[i];
        }

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
            if (Telemetry_Toggle_Check(&RC_Input_obj->OSD_Toggle).state)
            {
                if (Receiver_Obj.OSDTune_TriggerMs == 0)
                    Receiver_Obj.OSDTune_TriggerMs = SrvOsCommon.get_os_ms();

                if (SrvOsCommon.get_os_ms() - Receiver_Obj.OSDTune_TriggerMs >= TELEMETRY_OSDTUNE_POSHOLD)
                    RC_Input_obj->sig.osd_tune_state = true;
            }
            else
                Receiver_Obj.OSDTune_TriggerMs = 0;
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

        for (uint8_t i = Telemetry_RC_Throttle; i < Telemetry_Gimbal_TagSum; i++)
            RC_Input_obj->sig.gimbal_percent[i] = 0;

        Telemetry_Monitor.recover_failsafe = true;
        Telemetry_Led_Control(true);
    }

    memcpy(&sig_tmp, &RC_Input_obj->sig, sizeof(Telemetry_RCSig_TypeDef));

    return sig_tmp;
}

/************************************** telemetry radio section ********************************************/
static void Telemetry_DefaultPort_Init(Telemetry_PortMonitor_TypeDef *monitor)
{
    if(monitor)
    {
        if(BspUSB_VCP.init((uint32_t)&(monitor->VCP_Port.RecObj)) != BspUSB_Error_None)
        {
            /* init default port VCP first */
            monitor->VCP_Port.init_state = false;
        }
        else
            monitor->VCP_Port.init_state = true;

        /* create USB VCP Tx semaphore */
        osSemaphoreDef(DefaultPort_Tx);
        monitor->VCP_Port.p_tx_semphr = osSemaphoreCreate(osSemaphore(DefaultPort_Tx), 32);

        if(monitor->VCP_Port.p_tx_semphr == NULL)
        {
            monitor->VCP_Port.init_state = false;
            return BspUSB_Error_Semphr_Crt;
        }

        BspUSB_VCP.set_tx_cpl_callback(Telemetry_DefaultPort_TxCplt_Callback);
        BspUSB_VCP.set_rx_callback(Telemetry_Port_Rx_Callback);
    }
}

static void Telemetry_Port_Rx_Callback(uint32_t RecObj_addr, uint8_t *p_data, uint16_t size)
{
    SrvComProto_Msg_StreamIn_TypeDef stream_in;
    Telemetry_PortRecObj_TypeDef *p_RecObj = NULL;

    /* use mavlink protocol tuning the flight parameter */
    if(p_data && size && RecObj_addr)
    {
        p_RecObj = (Telemetry_PortRecObj_TypeDef *)RecObj_addr;
        p_RecObj->time_stamp = SrvOsCommon.get_os_ms();

        switch((uint8_t) p_RecObj->type)
        {
            case Telemetry_Port_USB:
                break;

            case Telemetry_Port_Uart:
                break;

            case Telemetry_Port_CAN:
                break;

            default:
                return;
        }

        stream_in = SrvComProto.msg_decode(p_data, size);
    
        if(stream_in.valid)
        {
            /* tag on recive time stamp */
            /* first come first serve */
            /* in case two different port tuning the same function or same parameter at the same time */
        }
    }
}

static void Telemetry_DefaultPort_TxCplt_Callback(uint8_t *p_data, uint32_t *size)
{
    if(PortMonitor.VCP_Port.p_tx_semphr)
    {
        if(osSemaphoreRelease(PortMonitor.VCP_Port.p_tx_semphr) != osOK)
        {
            PortMonitor.VCP_Port.tx_semphr_rls_err ++;
            osSemaphoreDelete(PortMonitor.VCP_Port.p_tx_semphr);
        }
    }
}

static void Telemetry_DefaultPort_Trans(uint8_t *p_data, uint16_t size)
{
    if(PortMonitor.VCP_Port.init_state && p_data && size)
    {
        osSemaphoreWait(PortMonitor.VCP_Port.p_tx_semphr, Telemetry_Port_Tx_TimeOut);

        if(BspUSB_VCP.send)
            BspUSB_VCP.send(p_data, size);
    }
}

static bool Telemetry_RadioPort_Init(void)
{
    /* USB VCP as defaut port to tune parameter and frame porotcol */
    if(!PortMonitor.init)
    {
        memset(&PortMonitor, 0, sizeof(PortMonitor));

        Telemetry_DefaultPort_Init(&PortMonitor);

        PortMonitor.init = true;
    }

    return false;
}

static bool Telemetry_RadioPort_ProtoManager()
{
    return false;
}

/************************************** telemetry radio section ********************************************/
static bool Telemetry_MAV_Msg_Init(void)
{
    /* create mavlink message object */
    if (SrvComProto.get_msg_type() == SrvComProto_Type_MAV)
    {
        SrvComProto_MavPackInfo_TypeDef PckInfo;

        memset(&PckInfo, 0, sizeof(PckInfo));
        memset(&TaskProto_MAV_RawIMU, 0, sizeof(TaskProto_MAV_RawIMU));
        memset(&TaskProto_MAV_ScaledIMU, 0, sizeof(TaskProto_MAV_ScaledIMU));
        memset(&TaskProto_MAV_RcChannel, 0, sizeof(TaskProto_MAV_RcChannel));
        memset(&TaskProto_MAV_MotoChannel, 0, sizeof(TaskProto_MAV_MotoChannel));
        memset(&TaskProto_MAV_Attitude, 0, sizeof(TaskProto_MAV_Attitude));

        // period 10Ms 100Hz
        PckInfo.system_id = MAV_SysID_Drone;
        PckInfo.component_id = MAV_CompoID_Raw_IMU;
        PckInfo.chan = 0;
        SrvComProto.mav_msg_obj_init(&TaskProto_MAV_RawIMU, PckInfo, 10);
        SrvComProto.mav_msg_enable_ctl(&TaskProto_MAV_RawIMU, true);

        // period 10Ms 100Hz
        PckInfo.system_id = MAV_SysID_Drone;
        PckInfo.component_id = MAV_CompoID_Scaled_IMU;
        PckInfo.chan = 0;
        SrvComProto.mav_msg_obj_init(&TaskProto_MAV_ScaledIMU, PckInfo, 10);
        SrvComProto.mav_msg_enable_ctl(&TaskProto_MAV_ScaledIMU, true);

        // period 20Ms 50Hz
        PckInfo.system_id = MAV_SysID_Drone;
        PckInfo.component_id = MAV_CompoID_RC_Channel;
        PckInfo.chan = 0;
        SrvComProto.mav_msg_obj_init(&TaskProto_MAV_RcChannel, PckInfo, 20);
        SrvComProto.mav_msg_enable_ctl(&TaskProto_MAV_RcChannel, true);

        // period 10Ms 100Hz
        PckInfo.system_id = MAV_SysID_Drone;
        PckInfo.component_id = MAV_CompoID_MotoCtl;
        PckInfo.chan = 0;
        SrvComProto.mav_msg_obj_init(&TaskProto_MAV_MotoChannel, PckInfo, 10);
        SrvComProto.mav_msg_enable_ctl(&TaskProto_MAV_MotoChannel, true);

        // period 20Ms 50Hz
        PckInfo.system_id = MAV_SysID_Drone;
        PckInfo.component_id = MAV_CompoID_Attitude;
        PckInfo.chan = 0;
        SrvComProto.mav_msg_obj_init(&TaskProto_MAV_Attitude, PckInfo, 20);
        SrvComProto.mav_msg_enable_ctl(&TaskProto_MAV_Attitude, true);

        return true;
    }

    return false;
}
