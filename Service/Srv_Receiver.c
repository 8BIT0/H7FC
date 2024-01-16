#include "Srv_Receiver.h"
#include "Bsp_Uart.h"
#include "Bsp_SPI.h"
#include "Bsp_GPIO.h"
#include "error_log.h"
#include "IO_Definition.h"
#include "Srv_OsCommon.h"

static uint8_t SrvReceiver_Buff[SRV_RECEIVER_BUFF_SIZE] __attribute__((section(".Perph_Section")));

static Error_Handler SrvReceiver_Error_Handle = NULL;
static SrvReceiver_Monitor_TypeDef SrvReceiver_Monitor;

/* internal function */
static void SrvReceiver_SerialDecode_Callback(SrvReceiverObj_TypeDef *obj, uint8_t *p_data, uint16_t size);
static bool SrvReceiver_Check(SrvReceiverObj_TypeDef *receiver_obj);
static void SrvReceiver_Set_Invert(SrvReceiverObj_TypeDef *receiver_obj, uint16_t channel_index);

/* external function */
static uint8_t *SrvReceiver_Create_UartObj(uint32_t serial_instance,
                                           uint32_t rx_dma,
                                           uint32_t rx_dma_stream,
                                           uint32_t tx_dma,
                                           uint32_t tx_dma_stream,
                                           bool swap,
                                           const BspGPIO_Obj_TypeDef tx_pin,
                                           const BspGPIO_Obj_TypeDef rx_pin);
static uint8_t *SrvReceiver_Create_SPIObj(void);
static bool SrvReceiver_Init(SrvReceiverObj_TypeDef *obj, uint8_t *port_obj);
static SrvReceiverData_TypeDef SrvReceiver_Get_Value(SrvReceiverObj_TypeDef *receiver_obj);
static bool SrvReceiver_Get_Scope(SrvReceiverObj_TypeDef *receiver_obj, int16_t *max, int16_t *mid, int16_t *min);

SrvReceiver_TypeDef SrvReceiver = {
    .create_serial_obj = SrvReceiver_Create_UartObj,
    .create_spi_obj = SrvReceiver_Create_SPIObj,
    .init = SrvReceiver_Init,
    .get = SrvReceiver_Get_Value,
    .invert = SrvReceiver_Set_Invert,
    .get_scope = SrvReceiver_Get_Scope,
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

static uint8_t *SrvReceiver_Create_UartObj(uint32_t serial_instance,
                                           uint32_t rx_dma,
                                           uint32_t rx_dma_stream,
                                           uint32_t tx_dma,
                                           uint32_t tx_dma_stream,
                                           bool swap,
                                           const BspGPIO_Obj_TypeDef tx_pin,
                                           const BspGPIO_Obj_TypeDef rx_pin)
{
    BspUARTObj_TypeDef *Uart_Receiver_Obj = NULL;

    Uart_Receiver_Obj = (BspUARTObj_TypeDef *)SrvOsCommon.malloc(sizeof(BspUARTObj_TypeDef));

    if (Uart_Receiver_Obj == NULL)
    {
        SrvOsCommon.free(Uart_Receiver_Obj);
        return NULL;
    }

    memset(Uart_Receiver_Obj, 0, sizeof(BspUARTObj_TypeDef));

    Uart_Receiver_Obj->instance = serial_instance;
    Uart_Receiver_Obj->tx_io = tx_pin;
    Uart_Receiver_Obj->rx_io = rx_pin;
    Uart_Receiver_Obj->pin_swap = swap;
    Uart_Receiver_Obj->rx_dma = rx_dma;
    Uart_Receiver_Obj->rx_stream = rx_dma_stream;
    Uart_Receiver_Obj->tx_dma = tx_dma;
    Uart_Receiver_Obj->tx_stream = tx_dma_stream;

    return Uart_Receiver_Obj;
}

static uint8_t *SrvReceiver_Create_SPIObj(void)
{
}

static bool SrvReceiver_Init(SrvReceiverObj_TypeDef *obj, uint8_t *port_obj)
{
    bool data_obj_error = false;
    BspUARTObj_TypeDef *Uart_Receiver_Obj = NULL;

    if ((obj == NULL) || (port_obj == NULL))
        return false;

    memset(SrvReceiver_Buff, NULL, SRV_RECEIVER_BUFF_SIZE);

    /* create error log handle */
    SrvReceiver_Error_Handle = ErrorLog.create("SrvReceiver_Error");

    /* regist all error to the error tree */
    ErrorLog.registe(SrvReceiver_Error_Handle, SrvReceiver_ErrorList, sizeof(SrvReceiver_ErrorList) / sizeof(SrvReceiver_ErrorList[0]));

    switch (obj->port_type)
    {
    case Receiver_Port_Serial:
        Uart_Receiver_Obj = (BspUARTObj_TypeDef *)port_obj;
        
        Uart_Receiver_Obj->hdl = SrvOsCommon.malloc(UART_HandleType_Size);
        if(Uart_Receiver_Obj->hdl == NULL)
        {
            SrvOsCommon.free(Uart_Receiver_Obj->hdl);
            return false;
        }

        Uart_Receiver_Obj->tx_dma_hdl = SrvOsCommon.malloc(UART_DMA_Handle_Size);
        if(Uart_Receiver_Obj->tx_dma_hdl)
        {
            SrvOsCommon.free(Uart_Receiver_Obj->tx_dma_hdl);
            SrvOsCommon.free(Uart_Receiver_Obj->hdl);
            return false;
        }

        Uart_Receiver_Obj->rx_dma_hdl = SrvOsCommon.malloc(UART_DMA_Handle_Size);
        if(Uart_Receiver_Obj->rx_dma_hdl)
        {
            SrvOsCommon.free(Uart_Receiver_Obj->rx_dma_hdl);
            SrvOsCommon.free(Uart_Receiver_Obj->tx_dma_hdl);
            SrvOsCommon.free(Uart_Receiver_Obj->hdl);
            return false;
        }

        memset(&SrvReceiver_Monitor, 0, SRVRECEIVER_SIZE);

        switch (obj->Frame_type)
        {
        case Receiver_Type_Sbus:
            Uart_Receiver_Obj->baudrate = SBUS_BAUDRATE;

            /* create data obj */
            obj->frame_data_obj = SrvOsCommon.malloc(sizeof(DevSBUSObj_TypeDef));
            if (obj->frame_data_obj == NULL)
            {
                data_obj_error = true;
                break;
            }

            /* init inverter pin */
            // obj->inverter_init();
            // obj->invert_control();

            /* set max channel num */
            obj->channel_num = SBUS_MAX_CHANNEL;

            /* set receiver object frame object */
            obj->frame_api = &DevSBUS;

            if (!((DevSBUS_TypeDef *)(obj->frame_api))->init(obj->frame_data_obj))
                return false;
            break;

        case Receiver_Type_CRSF:
            Uart_Receiver_Obj->baudrate = CRSF_BAUDRATE;

            /* create data obj */
            obj->frame_data_obj = SrvOsCommon.malloc(sizeof(DevCRSFObj_TypeDef));
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

            if (!((DevCRSF_TypeDef *)(obj->frame_api))->init(obj->frame_data_obj))
                return false;
            break;

        default:
            return false;
        }

        obj->data.val_list = (uint16_t *)SrvOsCommon.malloc(sizeof(uint16_t) * obj->channel_num);
        if (!obj->data.val_list)
        {
            SrvOsCommon.free(obj->data.val_list);
            return false;
        }

        obj->port = (SrvReceiver_Port_TypeDef *)SrvOsCommon.malloc(sizeof(SrvReceiver_Port_TypeDef));
        if (!obj->port)
        {
            SrvOsCommon.free(obj->data.val_list);
            return false;
        }

        if (data_obj_error)
        {
            ErrorLog.trigger(SrvReceiver_Error_Handle, Receiver_Obj_Error, &SrvReceiver_Monitor, SRVRECEIVER_SIZE);
            return false;
        }

        /* set uart init parameter */
        Uart_Receiver_Obj->rx_buf = SrvReceiver_Buff;
        Uart_Receiver_Obj->rx_size = SRV_RECEIVER_BUFF_SIZE;
        Uart_Receiver_Obj->cust_data_addr = obj;

        /* set uart callback */
        Uart_Receiver_Obj->RxCallback = SrvReceiver_SerialDecode_Callback;

        /* serial port init */
        if (!BspUart.init(Uart_Receiver_Obj))
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

    obj->invert_list = 0;

    return true;
}

static void SrvReceiver_SerialDecode_Callback(SrvReceiverObj_TypeDef *receiver_obj, uint8_t *p_data, uint16_t size)
{
    BspUARTObj_TypeDef *uart_obj = NULL;
    uint8_t *rx_buff_ptr = NULL;
    uint16_t rx_buff_size = 0;
    uint8_t decode_out = 0xFF;
    bool sig_update = false;

    if (receiver_obj && receiver_obj->frame_api && receiver_obj->port && receiver_obj->frame_data_obj)
    {
        if (receiver_obj->port_type == Receiver_Port_Serial)
        {
            /* do serial decode funtion */
            if (receiver_obj->Frame_type == Receiver_Type_CRSF)
            {
                decode_out = ((DevCRSF_TypeDef *)(receiver_obj->frame_api))->decode(receiver_obj->frame_data_obj, p_data, size);
                sig_update = true;

                switch (decode_out)
                {
                    case CRSF_FRAMETYPE_LINK_STATISTICS:
                        receiver_obj->data.rssi = ((DevCRSF_TypeDef *)(receiver_obj->frame_api))->get_statistics(receiver_obj->frame_data_obj).downlink_RSSI;
                        receiver_obj->data.link_quality = ((DevCRSF_TypeDef *)(receiver_obj->frame_api))->get_statistics(receiver_obj->frame_data_obj).downlink_Link_quality;
                        receiver_obj->data.active_antenna = ((DevCRSF_TypeDef *)(receiver_obj->frame_api))->get_statistics(receiver_obj->frame_data_obj).active_antenna;
                        break;

                    case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
                        ((DevCRSF_TypeDef *)(receiver_obj->frame_api))->get_channel(receiver_obj->frame_data_obj, receiver_obj->data.val_list);

                        for (uint8_t i = 0; i < receiver_obj->channel_num; i++)
                        {
                            if (receiver_obj->data.val_list[i] < CRSF_DIGITAL_CHANNEL_MIN)
                            {
                                receiver_obj->data.val_list[i] = CRSF_DIGITAL_CHANNEL_MIN;
                            }
                            else if (receiver_obj->data.val_list[i] > CRSF_DIGITAL_CHANNEL_MAX)
                            {
                                receiver_obj->data.val_list[i] = CRSF_DIGITAL_CHANNEL_MAX;
                            }

                            if (receiver_obj->invert_list && (receiver_obj->invert_list & 1 << i))
                            {
                                receiver_obj->data.val_list[i] -= CHANNEL_RANGE_MID;
                                receiver_obj->data.val_list[i] = CHANNEL_RANGE_MID - receiver_obj->data.val_list[i];
                            }
                        }

                        receiver_obj->data.failsafe = false;
                        break;

                    default:
                        sig_update = false;
                        return;
                }

                
            }
            else if (receiver_obj->Frame_type == Receiver_Type_Sbus)
            {
                if (((DevSBUS_TypeDef *)(receiver_obj->frame_api))->decode(receiver_obj->frame_data_obj, p_data, size) == DevSBUS_NoError)
                {
                    sig_update = true;
                    for (uint8_t i = 0; i < receiver_obj->channel_num; i++)
                    {
                        receiver_obj->data.val_list[i] = ((DevSBUSObj_TypeDef *)receiver_obj->frame_data_obj)->val[i];

                        if (receiver_obj->invert_list && receiver_obj->invert_list & 1 << i)
                        {
                            receiver_obj->data.val_list[i] -= CHANNEL_RANGE_MID;
                            receiver_obj->data.val_list[i] = CHANNEL_RANGE_MID - receiver_obj->data.val_list[i];
                        }
                    }
                }
            }

            if (sig_update && receiver_obj->in_use)
            {
                receiver_obj->re_update = true;
                
                /* set decode time stamp */
                receiver_obj->data.time_stamp = SrvOsCommon.get_os_ms();
            }
            else
                receiver_obj->re_update = false;            

            /* clear serial obj received data */
            if (receiver_obj->port->cfg)
            {
                uart_obj = ((BspUARTObj_TypeDef *)(receiver_obj->port->cfg));

                rx_buff_ptr = uart_obj->rx_buf;

                if (rx_buff_ptr)
                    memset(rx_buff_ptr, 0, size);
            }
        }
    }
}

static bool SrvReceiver_Check(SrvReceiverObj_TypeDef *receiver_obj)
{
    /* update check */
    /* update frequence less than 10hz switch into failsafe */
    if ((SrvOsCommon.get_os_ms() - receiver_obj->data.time_stamp) > SRV_RECEIVER_UPDATE_TIMEOUT_MS)
        return false;

    /* range check */
    for (uint8_t i = 0; i < receiver_obj->channel_num; i++)
    {
        if ((receiver_obj->data.val_list[i] > CHANNEL_RANGE_MAX) || (receiver_obj->data.val_list[i] < CHANNEL_RANGE_MIN))
            return false;
    }

    /* check RSSI Or Statistics Data */
    if (receiver_obj->Frame_type == Receiver_Type_CRSF)
    {
        /* check CRSF Statistics Data */
        // if ((receiver_obj->data.rssi < SRV_RECEIVER_MIN_RSSI) ||
        //     (receiver_obj->data.link_quality < SRV_RECEIVER_MIN_LINKQUALITY) ||
        //     (receiver_obj->data.active_antenna < SRV_RECEIVER_MIN_ANTENNA_VALUE))
        //    return false;
    }
    else if (receiver_obj->Frame_type == Receiver_Type_Sbus)
    {
        /* check SBUS Last Byte Functional Bit Or check RSSI */
        /* currently we don't have any sbus device for testing */
        /* return false currently */
        return false;
    }

    return true;
}

static void SrvReceiver_Set_Invert(SrvReceiverObj_TypeDef *receiver_obj, uint16_t channel_index)
{
    if (receiver_obj && (channel_index < receiver_obj->channel_num))
    {
        if (receiver_obj->invert_list & (1 << channel_index))
        {
            receiver_obj->invert_list &= ~(1 << channel_index);
        }
        else
        {
            receiver_obj->invert_list |= (1 << channel_index);
        }
    }
}

static SrvReceiverData_TypeDef SrvReceiver_Get_Value(SrvReceiverObj_TypeDef *receiver_obj)
{
    SrvReceiverData_TypeDef receiver_data_tmp;

    memset(&receiver_data_tmp, 0, sizeof(receiver_data_tmp));
    receiver_data_tmp.failsafe = true;

    if (receiver_obj == NULL)
        return receiver_data_tmp;

re_do:
    receiver_obj->in_use = true;

    if (!SrvReceiver_Check(receiver_obj))
        return receiver_data_tmp;

    receiver_data_tmp.failsafe = false;
    receiver_data_tmp = receiver_obj->data;

    receiver_obj->in_use = false;

    if (receiver_obj->re_update)
    {
        receiver_obj->re_update = false;
        goto re_do;
    }

    return receiver_data_tmp;
}

static bool SrvReceiver_Get_Scope(SrvReceiverObj_TypeDef *receiver_obj, int16_t *max, int16_t *mid, int16_t *min)
{
    if(receiver_obj && max && mid && min)
    {
        switch((uint8_t)receiver_obj->Frame_type)
        {
            case Receiver_Type_CRSF:
                (*max) = CRSF_DIGITAL_CHANNEL_MAX;
                (*mid) = CRSF_DIGITAL_CHANNEL_MID;
                (*min) = CRSF_DIGITAL_CHANNEL_MIN;
                return true;

            case Receiver_Type_Sbus:
                return false;

            default:
                return false;  
        }
    }

    return false;
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
