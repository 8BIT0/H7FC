#include "Srv_Receiver.h"
#include "Bsp_Uart.h"
#include "Bsp_SPI.h"
#include "error_log.h"
#include "IO_Definition.h"
#include "mmu.h"

__weak uint32_t SrvReceiver_Get_SysMs(void) { return 0; }

static uint8_t SrvReceiver_Buff[SRV_RECEIVER_BUFF_SIZE];

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

bool SrvReceiver_Init(SrvReceiverObj_TypeDef *obj)
{
    if (obj == NULL)
        return false;

    memset(SrvReceiver_Buff, NULL, SRV_RECEIVER_BUFF_SIZE);

    /* create error log handle */
    SrvReceiver_Error_Handle = ErrorLog.create("SrvReceiver_Error");

    /* regist all error to the error tree */
    ErrorLog.registe(SrvReceiver_Error_Handle, SrvReceiver_ErrorList, sizeof(SrvReceiver_ErrorList) / sizeof(SrvReceiver_ErrorList[0]));

    switch (obj->port_type)
    {
    case Receiver_Port_Serial:
        BspUARTObj_TypeDef *Uart_Receiver_Obj;

        Uart_Receiver_Obj = (BspUARTObj_TypeDef *)MMU_Malloc(sizeof(BspUARTObj_TypeDef));
        if(Uart_Receiver_Obj == NULL)
        {
            MMU_Free(Uart_Receiver_Obj);
            return false;
        }

        memset(Uart_Receiver_Obj, NULL, sizeof(Uart_Receiver_Obj));
        memset(&SrvReceiver_Monitor, NULL, SRVRECEIVER_SIZE);

        switch (obj->Frame_type)
        {
        case Receiver_Type_Sbus:
            Uart_Receiver_Obj->baudrate = SBUS_BAUDRATE;
            break;

        case Receiver_Type_CRSF:
            Uart_Receiver_Obj->baudrate = CRSF_BAUDRATE;
            break;

        default:
            return false;
        }

        Uart_Receiver_Obj->instance = UART4;
        Uart_Receiver_Obj->pin_swap = false;
        Uart_Receiver_Obj->rx_io = Uart4_RxPin;
        Uart_Receiver_Obj->tx_io = Uart4_TxPin;
        Uart_Receiver_Obj->rx_dma = Bsp_DMA_1;
        Uart_Receiver_Obj->rx_stream = Bsp_DMA_Stream_4;
        Uart_Receiver_Obj->tx_dma = Bsp_DMA_1;
        Uart_Receiver_Obj->tx_stream = Bsp_DMA_Stream_5;
        Uart_Receiver_Obj->rx_buf = SrvReceiver_Buff;
        Uart_Receiver_Obj->rx_size = SRV_RECEIVER_BUFF_SIZE;
        Uart_Receiver_Obj->cust_data_addr = (uint32_t)obj;

        /* set uart callback */

        /* serial port init */
        if(!BspUart.init(&Uart_Receiver_Obj))
        {
            ErrorLog.trigger(SrvReceiver_Error_Handle, Receiver_Port_Init_Error, &SrvReceiver_Monitor, SRVRECEIVER_SIZE);
            return false;
        }

        obj->port->api = &BspUart;
        obj->port->cfg = Uart_Receiver_Obj;
        break;

    case Receiver_Port_Spi:
        /* spi port init */
        break;

    default:
        ErrorLog.trigger(SrvReceiver_Error_Handle, Receiver_Obj_Error, &SrvReceiver_Monitor, SRVRECEIVER_SIZE);
        return false;
    }


    return true;
}

static void SrvReceiver_Decode_Callback(SrvReceiverObj_TypeDef *obj)
{
    if (obj && obj->cb)
    {
    }
}

void SrvReceiver_Range_Check()
{
}

/*************************************************************** Error Process Tree Callback *******************************************************************************/
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
