#include "Srv_Receiver.h"
#include "error_log.h"

__weak uint32_t SrvReceiver_Get_SysMs(void) { return 0; }

static Error_Handler SrvReceiver_Error_Handle = NULL;
static SrvReceiver_Monitor_TypeDef SrvReceiver_Monitor;

static const uint8_t default_channle_id_list[Receiver_Channel_Sum] = {
    Receiver_ChannelID_Pitch,
    Receiver_ChannelID_Roll,
    Receiver_ChannelID_Throttle,
    Receiver_ChannelID_Yaw,
    Receiver_ChannelID_AUX_1,
    Receiver_ChannelID_AUX_2,
    Receiver_ChannelID_AUX_3,
    Receiver_ChannelID_AUX_4,
    Receiver_ChannelID_AUX_5,
    Receiver_ChannelID_AUX_6,
    Receiver_ChannelID_AUX_7,
    Receiver_ChannelID_AUX_8,
    Receiver_ChannelID_AUX_9,
    Receiver_ChannelID_AUX_10,
    Receiver_ChannelID_AUX_11,
    Receiver_ChannelID_AUX_12,
};

static Error_Obj_Typedef SrvReceiver_ErrorList[] = {
    {
        .out = false,
        .log = false,
        .prc_callback = NULL,
        .code = Receiver_Obj_Error,
        .desc = "Receiver Object Input Error\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = false,
        .log = false,
        .prc_callback = NULL,
        .code = Receiver_Port_Error,
        .desc = "Receiver Port Error\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = false,
        .log = false,
        .prc_callback = NULL,
        .code = Receiver_Port_Init_Error,
        .desc = "Receiver Port Init Error\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = false,
        .log = false,
        .prc_callback = NULL,
        .code = Receiver_FrameType_Error,
        .desc = "Receiver Frame Type Error\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
};

bool SrvReceiver_Init(SrvReceiverObj_TypeDef *obj, void *port_ptr)
{
    if ((obj == NULL) || (port_ptr == NULL))
        return false;

    memset(&SrvReceiver_Monitor, NULL, SRVRECEIVER_SIZE);

    /* create error log handle */
    SrvReceiver_Error_Handle = ErrorLog.create("SrvReceiver_Error");

    /* regist all error to the error tree */
    ErrorLog.registe(SrvReceiver_Error_Handle, SrvReceiver_ErrorList, sizeof(SrvReceiver_ErrorList) / sizeof(SrvReceiver_ErrorList[0]));

    switch (obj->port_type)
    {
    case Receiver_Port_Serial:
        /* aserial port init */
        break;

    case Receiver_Port_Spi:
        /* spi port init */
        break;

    default:
        ErrorLog.trigger(SrvReceiver_Error_Handle, Receiver_Obj_Error, &SrvReceiver_Monitor, SRVRECEIVER_SIZE);
        return false;
    }

    switch (obj->Frame_type)
    {

    default:
        return false;
    }

    obj->port_ptr = port_ptr;

    return true;
}

void SrvReceiver_Decode_Callback(SrvReceiverObj_TypeDef *obj)
{
    if (obj && obj->cb)
    {
    }
}

void SrvReceiver_Range_Check()
{
}

/*************************************************************** Error Process Callback *******************************************************************************/
static void SrvReceiver_Obj_Error(int16_t code, uint8_t *p_arg, uint16_t size)
{
}

static void SrvReceiver_FrameType_Error(int16_t code, uint8_t *p_arg, uint16_t size)
{
}

static void SrvReceiver_Port_Error(int16_t code, uint8_t *p_arg, uint16_t size)
{
}

static void SrvReceiver_Port_Init_Error(int16_t code, uint8_t *p_arg, uint16_t size)
{
}

/*************************************************************** Directly Error Process *******************************************************************************/
static void SrvReceiver_UpdateTimeOut_Error()
{
}

static void SrvReceiver_ValueOverRange_Error()
{
}

static void SrvReceiver_ValueStepBump_Error()
{
}

static void SrvReceiver_NoneSignal_Error()
{
}
