#include "Srv_Receiver.h"
#include "Bsp_Uart.h"
#include "Bsp_SPI.h"
#include "Bsp_GPIO.h" 
#include "error_log.h"
#include "IO_Definition.h"
#include "mmu.h"

__weak uint32_t SrvReceiver_Get_SysMs(void) { return 0; }

static uint8_t SrvReceiver_Buff[SRV_RECEIVER_BUFF_SIZE];

static Error_Handler SrvReceiver_Error_Handle = NULL;
static SrvReceiver_Monitor_TypeDef SrvReceiver_Monitor;

/* internal function */
static void SrvReceiver_Decode_Callback(SrvReceiverObj_TypeDef *obj, uint8_t *p_data, uint16_t size);

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
    bool data_obj_error = false;
    BspUARTObj_TypeDef *Uart_Receiver_Obj;

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
        Uart_Receiver_Obj = (BspUARTObj_TypeDef *)MMU_Malloc(sizeof(BspUARTObj_TypeDef));
        if (Uart_Receiver_Obj == NULL)
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

            /* create data obj */
            obj->frame_data_obj = MMU_Malloc(sizeof(DevSBUSObj_TypeDef));
            if (obj->frame_data_obj == NULL)
            {
                data_obj_error = true;
                break;
            }

            /* init inverter pin */

            /* set max channel num */
            obj->channel_num = SBUS_MAX_CHANNEL;

            /* set receiver object frame object */
            obj->frame_api = &DevSBUS;

            if(!((DevSBUS_TypeDef *)(obj->frame_api))->init(obj->frame_data_obj))
                return false;
            break;

        case Receiver_Type_CRSF:
            Uart_Receiver_Obj->baudrate = CRSF_BAUDRATE;

            /* create data obj */
            obj->frame_data_obj = MMU_Malloc(sizeof(DevCRSFObj_TypeDef));
            if (obj->frame_data_obj == NULL)
            {
                data_obj_error = true;
                break;
            }

            /* for crsf receiver we dont need inverter */
            obj->invert_control = NULL;
            obj->inverter_init = NULL;

            /* set max channel num */
            obj->channel_num = CRSF_MAX_CHANNEL;

            /* set receiver object frame object */
            obj->frame_api = &DevCRSF;

            if(!((DevCRSF_TypeDef *)(obj->frame_api))->init(obj->frame_data_obj))
                return false;
            break;

        default:
            return false;
        }

        obj->update_freq = 0;

        if (data_obj_error)
        {
            ErrorLog.trigger(SrvReceiver_Error_Handle, Receiver_Obj_Error, &SrvReceiver_Monitor, SRVRECEIVER_SIZE);
            return false;
        }

        /* set uart init parameter */
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
        Uart_Receiver_Obj->cust_data_addr = obj;

        /* set uart callback */
        Uart_Receiver_Obj->RxCallback = SrvReceiver_Decode_Callback;

        /* serial port init */
        if (!BspUart.init(&Uart_Receiver_Obj))
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

static void SrvReceiver_Decode_Callback(SrvReceiverObj_TypeDef *receiver_obj, uint8_t *p_data, uint16_t size)
{
    BspUARTObj_TypeDef *uart_obj = NULL;
    uint8_t *rx_buff_ptr = NULL;
    uint16_t rx_buff_size = 0;
    uint8_t decode_out = 0xFF;

    if (receiver_obj && receiver_obj->frame_api && receiver_obj->port && receiver_obj->frame_data_obj)
    {
        if (receiver_obj->port_type == Receiver_Port_Serial)
        {
            /* do serial decode funtion */
            if(receiver_obj->Frame_type == Receiver_Type_CRSF)
            {
                crsf_channels_t crsf_frame_channel;

                memset(&crsf_frame_channel, NULL, sizeof(crsf_frame_channel));
                decode_out = ((DevCRSF_TypeDef *)(receiver_obj->frame_api))->decode(receiver_obj->frame_data_obj, p_data, size);

                switch(decode_out)
                {
                    case CRSF_FRAMETYPE_LINK_STATISTICS:
                    receiver_obj->data.rssi = ((DevCRSF_TypeDef *)(receiver_obj->frame_api))->get_statistics(receiver_obj->frame_data_obj).uplink_RSSI_1;
                    receiver_obj->data.link_quality = ((DevCRSF_TypeDef *)(receiver_obj->frame_api))->get_statistics(receiver_obj->frame_data_obj).uplink_Link_quality;
                    break;

                    case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
                    crsf_frame_channel = ((DevCRSF_TypeDef *)(receiver_obj->frame_api))->get_channel(receiver_obj->frame_data_obj);

                    receiver_obj->data.val_list[0] = crsf_frame_channel.ch0;
                    receiver_obj->data.val_list[1] = crsf_frame_channel.ch1;
                    receiver_obj->data.val_list[2] = crsf_frame_channel.ch2;
                    receiver_obj->data.val_list[3] = crsf_frame_channel.ch3;
                    receiver_obj->data.val_list[4] = crsf_frame_channel.ch4;
                    receiver_obj->data.val_list[5] = crsf_frame_channel.ch5;
                    receiver_obj->data.val_list[6] = crsf_frame_channel.ch6;
                    receiver_obj->data.val_list[7] = crsf_frame_channel.ch7;
                    receiver_obj->data.val_list[8] = crsf_frame_channel.ch8;
                    receiver_obj->data.val_list[9] = crsf_frame_channel.ch9;
                    receiver_obj->data.val_list[10] = crsf_frame_channel.ch10;
                    receiver_obj->data.val_list[11] = crsf_frame_channel.ch11;
                    receiver_obj->data.val_list[12] = crsf_frame_channel.ch12;
                    receiver_obj->data.val_list[13] = crsf_frame_channel.ch13;
                    receiver_obj->data.val_list[14] = crsf_frame_channel.ch14;
                    receiver_obj->data.val_list[15] = crsf_frame_channel.ch15;

                    receiver_obj->data.failsafe = false;
                    break;

                    default:
                        return;
                }
            }
            else if(receiver_obj->Frame_type == Receiver_Type_Sbus)
            {
                if(((DevSBUS_TypeDef *)(receiver_obj->frame_api))->decode(receiver_obj->frame_data_obj, p_data, size) == DevSBUS_NoError)
                {
                    for(uint8_t i = 0; i < receiver_obj->channel_num; i++)
                    {
                        receiver_obj->data.val_list[i] = ((DevSBUSObj_TypeDef *)receiver_obj->frame_data_obj)->val[i];
                    }
                }
            }

            /* set decode time stamp */
            receiver_obj->data.time_stamp = SrvReceiver_Get_SysMs();

            /* clear serial obj received data */
            if (receiver_obj->port->cfg)
            {
                uart_obj = ((BspUARTObj_TypeDef *)(receiver_obj->port->cfg));

                rx_buff_ptr = uart_obj->rx_buf;
                rx_buff_size = uart_obj->rx_size;

                if (rx_buff_ptr && rx_buff_size)
                    memset(rx_buff_ptr, NULL, rx_buff_size);
            }
        }
    }
}

static SrvReceiverData_TypeDef SrvReceiver_Get_Value(const SrvReceiverObj_TypeDef receiver_obj)
{
    return receiver_obj.data;
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
