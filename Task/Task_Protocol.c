#include "Task_Protocol.h"
#include "Srv_OsCommon.h"
#include "IO_Definition.h"
#include "Srv_ComProto.h"
#include "Srv_DataHub.h"

#define RADIO_TX_PIN UART1_TX_PIN
#define RADIO_RX_PIN UART1_RX_PIN

#define RADIO_TX_PIN_ALT GPIO_AF7_USART1
#define RADIO_RX_PIN_ALT GPIO_AF7_USART1

#define RADIO_TX_PORT UART1_TX_PORT
#define RADIO_RX_PORT UART1_RX_PORT

#if (RADIO_UART_NUM > 0)
static uint8_t RadioRxBuff[RADIO_UART_NUM][RADIO_BUFF_SIZE];

static BspUARTObj_TypeDef Radio_Port1_UartObj = {
    .instance = RADIO_PORT,
    .baudrate = RADIO_PORT_BAUD,
    .tx_io = {
        .init_state = false,
        .pin = RADIO_TX_PIN,
        .port = RADIO_TX_PORT,
        .alternate = RADIO_TX_PIN_ALT,
    }, 
    .rx_io = {
        .init_state = false,
        .pin = RADIO_RX_PIN,
        .port = RADIO_RX_PORT,
        .alternate = RADIO_RX_PIN_ALT,
    }, 
    .pin_swap = false,
    .rx_dma = RADIO_RX_DMA,
    .rx_stream = RADIO_RX_DMA_STREAM,
    .tx_dma = RADIO_TX_DMA,
    .tx_stream = RADIO_TX_DMA_STREAM,
    .rx_buf = RadioRxBuff[RADIO_UART_NUM],
    .rx_size = RADIO_BUFF_SIZE,
};

static FrameCTL_UartPortMonitor_TypeDef Radio_UartPort_List[RADIO_UART_NUM] = {
    [0] = {.init_state = false,
           .Obj = &Radio_Port1_UartObj},
};
#endif

#ifndef RADIO_CAN_NUM
#define RADIO_CAN_NUM 0
#endif

#if (RADIO_CAN_NUM > 0)
static FrameCTL_CanPortMonitor_TypeDef Radio_CANPort_List[RADIO_CAN_NUM];
#endif

/* internal variable */
/* MAVLink message List */
SrvComProto_MsgInfo_TypeDef TaskProto_MAV_RawIMU;
SrvComProto_MsgInfo_TypeDef TaskProto_MAV_ScaledIMU;
SrvComProto_MsgInfo_TypeDef TaskProto_MAV_RcChannel;
SrvComProto_MsgInfo_TypeDef TaskProto_MAV_MotoChannel;
SrvComProto_MsgInfo_TypeDef TaskProto_MAV_Attitude;
SrvComProto_MsgInfo_TypeDef TaskProto_MAV_Exp_Attitude;
SrvComProto_MsgInfo_TypeDef TaskProto_MAV_Altitude;

SrvComProto_MsgInfo_TypeDef RadioProto_MAV_RawIMU;
SrvComProto_MsgInfo_TypeDef RadioProto_MAV_ScaledIMU;
SrvComProto_MsgInfo_TypeDef RadioProto_MAV_RcChannel;
SrvComProto_MsgInfo_TypeDef RadioProto_MAV_MotoChannel;
SrvComProto_MsgInfo_TypeDef RadioProto_MAV_Attitude;
SrvComProto_MsgInfo_TypeDef RadioProto_MAV_Altitude;

static FrameCTL_Monitor_TypeDef FrameCTL_Monitor;
static bool FrameCTL_MavProto_Enable = false;
static FrameCTL_PortMonitor_TypeDef PortMonitor = {.init = false};
static uint32_t FrameCTL_Period = 0;
static __attribute__((section(".Perph_Section"))) uint8_t MavShareBuf[1024];
static __attribute__((section(".Perph_Section"))) uint8_t CLIRxBuf[CLI_FUNC_BUF_SIZE];
static uint8_t CLIProcBuf[CLI_FUNC_BUF_SIZE];
static uint32_t Radio_Addr = 0;
static uint32_t USB_VCP_Addr = 0;

static SrvComProto_Stream_TypeDef MavStream = {
    .p_buf = MavShareBuf,
    .size = 0,
    .max_size = sizeof(MavShareBuf),
};

static SrvComProto_Stream_TypeDef CLI_RX_Stream = {
    .p_buf = CLIRxBuf,
    .size = 0,
    .max_size = sizeof(CLIRxBuf),
};

static SrvComProto_Stream_TypeDef CLI_Proc_Stream = {
    .p_buf = CLIProcBuf,
    .size = 0,
    .max_size = sizeof(CLIProcBuf),
};

static FrameCTL_CLIMonitor_TypeDef CLI_Monitor = {
    .port_addr = 0,
    .p_rx_stream = &CLI_RX_Stream,
    .p_proc_stream = &CLI_Proc_Stream,
};

/* frame section */
static void TaskFrameCTL_PortFrameOut_Process(void);
static void TaskFrameCTL_MavMsg_Trans(FrameCTL_Monitor_TypeDef *Obj, uint8_t *p_data, uint16_t size);

/* default vcp port section */
static void TaskFrameCTL_DefaultPort_Init(FrameCTL_PortMonitor_TypeDef *monitor);
static void TaskFrameCTL_RadioPort_Init(FrameCTL_PortMonitor_TypeDef *monitor);
static bool TaskFrameCTL_MAV_Msg_Init(void);
static void TaskFrameCTL_Port_Rx_Callback(uint32_t RecObj_addr, uint8_t *p_data, uint16_t size);
static void TaskFrameCTL_Port_TxCplt_Callback(uint32_t RecObj_addr, uint8_t *p_data, uint32_t *size);
static uint32_t TaskFrameCTL_Set_RadioPort(FrameCTL_PortType_List port_type, uint16_t index);
static void TaskFrameCTL_Port_Tx(uint32_t obj_addr, uint8_t *p_data, uint16_t size);
static void TaskFrameCTL_ConnectStateCheck(void);
static void TaskFrameCTL_CLI_Proc(void);
static void TaskFrameCTL_CLI_Trans(uint8_t *p_data, uint16_t size);

void TaskFrameCTL_Init(uint32_t period)
{
    FrameCTL_Period = FrameCTL_MAX_Period;

    /* USB VCP as defaut port to tune parameter and frame porotcol */
    memset(&PortMonitor, 0, sizeof(PortMonitor));

    TaskFrameCTL_DefaultPort_Init(&PortMonitor);
    // TaskFrameCTL_RadioPort_Init(&PortMonitor);
    Radio_Addr = TaskFrameCTL_Set_RadioPort(Port_Uart, 0);

    PortMonitor.init = true;
    
    /* init radio protocol*/
    FrameCTL_MavProto_Enable = TaskFrameCTL_MAV_Msg_Init();

    if(period && (period <= FrameCTL_MAX_Period))
    {
        FrameCTL_Period = period;
    }

    /* Shell Init */
    Shell_Init(TaskFrameCTL_CLI_Trans, CLI_Monitor.p_proc_stream->p_buf, CLI_Monitor.p_proc_stream->max_size);
}

void TaskFrameCTL_Core(void *arg)
{
    uint32_t per_time = SrvOsCommon.get_os_ms();

    while(1)
    {
        /* frame protocol process */
        TaskFrameCTL_PortFrameOut_Process();

        /* command line process */
        TaskFrameCTL_CLI_Proc();

        TaskFrameCTL_ConnectStateCheck();

        SrvOsCommon.precise_delay(&per_time, FrameCTL_Period);
    }
}

/*************************** ByPass Mode is still in developping *********************************/
/************************************** radio section ********************************************/
static void TaskFrameCTL_DefaultPort_Init(FrameCTL_PortMonitor_TypeDef *monitor)
{
    if(monitor)
    {
        if(BspUSB_VCP.init((uint32_t)&(monitor->VCP_Port.RecObj)) != BspUSB_Error_None)
        {
            /* init default port VCP first */
            monitor->VCP_Port.init_state = false;
            return;
        }
        else
            monitor->VCP_Port.init_state = true;

        /* create USB VCP Tx semaphore */
        osSemaphoreDef(DefaultPort_Tx);
        monitor->VCP_Port.p_tx_semphr = osSemaphoreCreate(osSemaphore(DefaultPort_Tx), 32);

        if(monitor->VCP_Port.p_tx_semphr == NULL)
        {
            monitor->VCP_Port.init_state = false;
            return;
        }

        BspUSB_VCP.set_tx_cpl_callback(TaskFrameCTL_Port_TxCplt_Callback);
        BspUSB_VCP.set_rx_callback(TaskFrameCTL_Port_Rx_Callback);

        monitor->VCP_Port.RecObj.PortObj_addr = (uint32_t)&(monitor->VCP_Port);

        USB_VCP_Addr = (uint32_t)&(monitor->VCP_Port);
    }
}

static void TaskFrameCTL_DefaultPort_Trans(uint8_t *p_data, uint16_t size)
{
    if(PortMonitor.VCP_Port.init_state && PortMonitor.VCP_Port.p_tx_semphr && p_data && size)
    {
        osSemaphoreWait(PortMonitor.VCP_Port.p_tx_semphr, FrameCTL_Port_Tx_TimeOut);

        if(BspUSB_VCP.send)
            BspUSB_VCP.send(p_data, size);
    }
}

/************************************** radio port section *************************/
static void TaskFrameCTL_RadioPort_Init(FrameCTL_PortMonitor_TypeDef *monitor)
{
    if(monitor)
    {
#if (RADIO_UART_NUM > 0)
        monitor->uart_port_num = RADIO_UART_NUM;
        monitor->Uart_Port = Radio_UartPort_List;
        
        for(uint8_t i = 0; i < monitor->uart_port_num; i++)
        {
            /* create port obj element */
            monitor->Uart_Port[i].Obj->hdl = SrvOsCommon.malloc(UART_HandleType_Size);
            if(monitor->Uart_Port[i].Obj->hdl == NULL)
            {
                SrvOsCommon.free(monitor->Uart_Port[i].Obj->hdl);
                return false;
            }

            monitor->Uart_Port[i].Obj->rx_dma_hdl = SrvOsCommon.malloc(UART_DMA_Handle_Size);
            if(monitor->Uart_Port[i].Obj->rx_dma_hdl == NULL)
            {
                SrvOsCommon.free(monitor->Uart_Port[i].Obj->rx_dma_hdl);
                SrvOsCommon.free(monitor->Uart_Port[i].Obj->hdl);
                return false;
            }

            monitor->Uart_Port[i].Obj->tx_dma_hdl = SrvOsCommon.malloc(UART_DMA_Handle_Size);
            if(monitor->Uart_Port[i].Obj->tx_dma_hdl == NULL)
            {
                SrvOsCommon.free(monitor->Uart_Port[i].Obj->rx_dma_hdl);
                SrvOsCommon.free(monitor->Uart_Port[i].Obj->tx_dma_hdl);
                SrvOsCommon.free(monitor->Uart_Port[i].Obj->hdl);
                return false;
            }


            if(BspUart.init(monitor->Uart_Port[i].Obj))
            {
                monitor->Uart_Port[i].init_state = true;
                memset(&monitor->Uart_Port[i].RecObj, 0, sizeof(FrameCTL_PortProtoObj_TypeDef));
                memset(&monitor->Uart_Port[i].ByPass_Mode, 0, sizeof(Port_Bypass_TypeDef));
                
                monitor->Uart_Port[i].RecObj.type = Port_Uart;
                monitor->Uart_Port[i].RecObj.port_index = i;
                
                monitor->Uart_Port[i].Obj->cust_data_addr = (uint32_t)&(monitor->Uart_Port[i].RecObj);
            
                /* create semaphore for send */
                osSemaphoreDef(Uart_Port_Tmp);
                monitor->Uart_Port[i].p_tx_semphr = osSemaphoreCreate(osSemaphore(Uart_Port_Tmp), 32);

                if(monitor->Uart_Port[i].p_tx_semphr)
                {
                    /* set callback */
                    BspUart.set_rx_callback(monitor->Uart_Port[i].Obj, TaskFrameCTL_Port_Rx_Callback);
                    BspUart.set_tx_callback(monitor->Uart_Port[i].Obj, TaskFrameCTL_Port_TxCplt_Callback);

                    monitor->Uart_Port[i].RecObj.PortObj_addr = (uint32_t)&(monitor->Uart_Port[i]);
                    monitor->Uart_Port[i].init_state = true;
                }
                else
                {
                    monitor->Uart_Port[i].init_state = false;
                    monitor->uart_port_num --;
                }
            }
            else
            {
                monitor->Uart_Port[i].init_state = false;
                monitor->uart_port_num --;
            }
        }
#else
        monitor->uart_port_num = 0;
#endif
    }
}

static uint32_t TaskFrameCTL_Set_RadioPort(FrameCTL_PortType_List port_type, uint16_t index)
{
    uint32_t port_hdl = 0;

    switch((uint8_t) port_type)
    {
        case Port_Uart:
            if((index < PortMonitor.uart_port_num) && PortMonitor.Uart_Port[index].init_state)
            {
                port_hdl = &(PortMonitor.Uart_Port[index]);
            }
            break;

        default:
            return port_hdl;
    }

    return port_hdl;
}

/************************************** receive process callback section *************************/
static void TaskFrameCTL_Port_Tx(uint32_t obj_addr, uint8_t *p_data, uint16_t size)
{
    FrameCTL_UartPortMonitor_TypeDef *p_UartPort = NULL;

    if(obj_addr && p_data && size)
    {
        p_UartPort = (FrameCTL_UartPortMonitor_TypeDef *)obj_addr;

        if(p_UartPort->init_state && p_UartPort->Obj && p_UartPort->p_tx_semphr)
        {
            osSemaphoreWait(p_UartPort->p_tx_semphr, FrameCTL_Port_Tx_TimeOut);
            BspUart.send(p_UartPort->Obj, p_data, size);
        }
    }
}

static void TaskFrameCTL_Port_Rx_Callback(uint32_t RecObj_addr, uint8_t *p_data, uint16_t size)
{
    SrvComProto_Msg_StreamIn_TypeDef stream_in;
    FrameCTL_PortProtoObj_TypeDef *p_RecObj = NULL;
    uint32_t port_addr = 0;
    bool cli_state = false;

    /* use mavlink protocol tuning the flight parameter */
    if(p_data && size && RecObj_addr)
    {
        SrvDataHub.get_cli_state(&cli_state);

        p_RecObj = (FrameCTL_PortProtoObj_TypeDef *)RecObj_addr;
        p_RecObj->time_stamp = SrvOsCommon.get_os_ms();

        switch((uint8_t) p_RecObj->type)
        {
            case Port_USB:
                break;

            case Port_Uart:
                break;

            case Port_CAN:
                break;

            default:
                return;
        }

        stream_in = SrvComProto.msg_decode(p_data, size);
    
        /* noticed when drone is under disarmed state we can`t tune or send cli to drone for safety */
        if(stream_in.valid)
        {
            /* tag on recive time stamp */
            /* first come first serve */
            /* in case two different port tuning the same function or same parameter at the same time */
            /* if attach to configrator or in tunning then lock moto */
            if(!cli_state && (stream_in.pac_type == ComFrame_MavMsg))
            {
                /* check mavline message frame type */
                /* only process mavlink message when cli is disabled */
            }
            else if(stream_in.pac_type == ComFrame_CLI)
            {
                /* set current mode as cli mode */
                /* all command line end up with "\r\n" */
                /* push string into cli shared stream */
                if(((CLI_Monitor.port_addr == 0) || (CLI_Monitor.port_addr == p_RecObj->PortObj_addr)) && \
                   (CLI_Monitor.p_rx_stream->size + size) <= CLI_Monitor.p_rx_stream->max_size)
                {

                    CLI_Monitor.type = p_RecObj->type;
                    CLI_Monitor.port_addr = p_RecObj->PortObj_addr;
                    memcpy(CLI_Monitor.p_rx_stream->p_buf + CLI_Monitor.p_rx_stream->size, stream_in.p_buf, stream_in.size);
                    CLI_Monitor.p_rx_stream->size += size;
                }
            }
        }
    }
}

static void TaskFrameCTL_Port_TxCplt_Callback(uint32_t Obj_addr, uint8_t *p_data, uint32_t *size)
{
    UNUSED(p_data);
    UNUSED(size);

    FrameCTL_PortProtoObj_TypeDef *p_Obj = NULL;
    FrameCTL_UartPortMonitor_TypeDef *p_UartPortObj = NULL;
    FrameCTL_VCPPortMonitor_TypeDef *p_USBPortObj = NULL;
    osSemaphoreId semID = NULL;
    uint32_t *p_rls_err_cnt = NULL;

    if(Obj_addr)
    {
        p_Obj = Obj_addr;

        if(p_Obj->PortObj_addr)
        {
            switch((uint8_t) p_Obj->type)
            {
                case Port_USB:
                    p_USBPortObj = (FrameCTL_VCPPortMonitor_TypeDef *)(p_Obj->PortObj_addr);

                    if(p_USBPortObj->init_state && p_USBPortObj->p_tx_semphr)
                    {
                        semID = p_USBPortObj->p_tx_semphr;
                        p_rls_err_cnt = &p_USBPortObj->tx_semphr_rls_err;
                    }
                    break;

                case Port_Uart:
                    p_UartPortObj = (FrameCTL_UartPortMonitor_TypeDef *)(p_Obj->PortObj_addr);

                    if(p_UartPortObj->init_state && p_UartPortObj->p_tx_semphr)
                    {
                        semID =p_UartPortObj->p_tx_semphr;
                        p_rls_err_cnt = &p_UartPortObj->tx_semphr_rls_err;
                    }
                    break;

                default:
                    return;
            }
            
            if(semID && p_rls_err_cnt && (osSemaphoreRelease(semID) != osOK))
                (*p_rls_err_cnt) ++;
        }
    }
}

/************************************** frame protocol section ********************************************/
static bool TaskFrameCTL_MAV_Msg_Init(void)
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
 
        memset(&PckInfo, 0, sizeof(PckInfo));
        memset(&RadioProto_MAV_RawIMU, 0, sizeof(TaskProto_MAV_RawIMU));
        memset(&RadioProto_MAV_ScaledIMU, 0, sizeof(TaskProto_MAV_ScaledIMU));
        memset(&RadioProto_MAV_RcChannel, 0, sizeof(TaskProto_MAV_RcChannel));
        memset(&RadioProto_MAV_MotoChannel, 0, sizeof(TaskProto_MAV_MotoChannel));
        memset(&RadioProto_MAV_Attitude, 0, sizeof(TaskProto_MAV_Attitude));
 
        // period 10Ms 100Hz
        PckInfo.system_id = MAV_SysID_Drone;
        PckInfo.component_id = MAV_CompoID_Raw_IMU;
        PckInfo.chan = 0;
        SrvComProto.mav_msg_obj_init(&TaskProto_MAV_RawIMU, PckInfo, 10);
        SrvComProto.mav_msg_enable_ctl(&TaskProto_MAV_RawIMU, true);
 
        SrvComProto.mav_msg_obj_init(&RadioProto_MAV_RawIMU, PckInfo, 10);
        SrvComProto.mav_msg_enable_ctl(&RadioProto_MAV_RawIMU, true);
             
        // period 10Ms 100Hz
        PckInfo.system_id = MAV_SysID_Drone;
        PckInfo.component_id = MAV_CompoID_Scaled_IMU;
        PckInfo.chan = 0;
        SrvComProto.mav_msg_obj_init(&TaskProto_MAV_ScaledIMU, PckInfo, 10);
        SrvComProto.mav_msg_enable_ctl(&TaskProto_MAV_ScaledIMU, true);

        SrvComProto.mav_msg_obj_init(&RadioProto_MAV_ScaledIMU, PckInfo, 10);
        SrvComProto.mav_msg_enable_ctl(&RadioProto_MAV_ScaledIMU, true);
        
        // period 20Ms 50Hz
        PckInfo.system_id = MAV_SysID_Drone;
        PckInfo.component_id = MAV_CompoID_RC_Channel;
        PckInfo.chan = 0;
        SrvComProto.mav_msg_obj_init(&TaskProto_MAV_RcChannel, PckInfo, 20);
        SrvComProto.mav_msg_enable_ctl(&TaskProto_MAV_RcChannel, true);

        SrvComProto.mav_msg_obj_init(&RadioProto_MAV_RcChannel, PckInfo, 20);
        SrvComProto.mav_msg_enable_ctl(&RadioProto_MAV_RcChannel, true);
        
        // period 10Ms 100Hz
        PckInfo.system_id = MAV_SysID_Drone;
        PckInfo.component_id = MAV_CompoID_MotoCtl;
        PckInfo.chan = 0;
        SrvComProto.mav_msg_obj_init(&TaskProto_MAV_MotoChannel, PckInfo, 10);
        SrvComProto.mav_msg_enable_ctl(&TaskProto_MAV_MotoChannel, true);

        SrvComProto.mav_msg_obj_init(&RadioProto_MAV_MotoChannel, PckInfo, 10);
        SrvComProto.mav_msg_enable_ctl(&RadioProto_MAV_MotoChannel, true);
        
        // period 20Ms 50Hz
        PckInfo.system_id = MAV_SysID_Drone;
        PckInfo.component_id = MAV_CompoID_Attitude;
        PckInfo.chan = 0;
        SrvComProto.mav_msg_obj_init(&TaskProto_MAV_Attitude, PckInfo, 20);
        SrvComProto.mav_msg_enable_ctl(&TaskProto_MAV_Attitude, true);

        SrvComProto.mav_msg_obj_init(&RadioProto_MAV_Attitude, PckInfo, 20);
        SrvComProto.mav_msg_enable_ctl(&RadioProto_MAV_Attitude, true);
               
        // period 20Ms 50Hz
        PckInfo.system_id = MAV_SysID_Drone;
        PckInfo.component_id = MAV_CompoID_Exp_Attitude;
        PckInfo.chan = 0;
        SrvComProto.mav_msg_obj_init(&TaskProto_MAV_Exp_Attitude, PckInfo, 20);
        SrvComProto.mav_msg_enable_ctl(&TaskProto_MAV_Exp_Attitude, true);
 
        // period 100MS 50Hz
        PckInfo.system_id = MAV_SysID_Drone;
        PckInfo.component_id = MAV_CompoID_Altitude;
        PckInfo.chan = 0;
        SrvComProto.mav_msg_obj_init(&TaskProto_MAV_Altitude, PckInfo, 20);
        SrvComProto.mav_msg_enable_ctl(&TaskProto_MAV_Altitude, true);

        SrvComProto.mav_msg_obj_init(&RadioProto_MAV_Altitude, PckInfo, 20);
        SrvComProto.mav_msg_enable_ctl(&RadioProto_MAV_Altitude, true);

        return true;
    }

    return false;
}

static void TaskFrameCTL_PortFrameOut_Process(void)
{
    FrameCTL_Monitor_TypeDef proto_monitor;
    void *proto_arg = (void *)&proto_monitor;
    bool tunning_state = false;
    uint32_t tunning_time_stamp = 0;
    uint32_t tunning_port = 0;
    bool arm_state = false;
    bool CLI_state = false;

    proto_monitor.frame_type = ComFrame_MavMsg;
    SrvDataHub.get_cli_state(&CLI_state);

    if(FrameCTL_MavProto_Enable && PortMonitor.VCP_Port.init_state && !CLI_state)
    {
        /* when attach to configrator then disable radio port trans use default port trans mav data */
        /* check other port init state */

        /* if in tunning than halt general frame protocol */
        SrvDataHub.get_tunning_state(&tunning_time_stamp, &tunning_state, &tunning_port);
        SrvDataHub.get_arm_state(&arm_state);

        if(!tunning_state)
        {
            /* Proto mavlink message through Radio */
            proto_monitor.port_type = Port_Uart;
            proto_monitor.port_addr = Radio_Addr;
            // SrvComProto.mav_msg_stream(&RadioProto_MAV_RawIMU,    &MavStream, proto_arg, (ComProto_Callback)TaskFrameCTL_MavMsg_Trans);
            SrvComProto.mav_msg_stream(&RadioProto_MAV_ScaledIMU, &MavStream, proto_arg, (ComProto_Callback)TaskFrameCTL_MavMsg_Trans);
            SrvComProto.mav_msg_stream(&RadioProto_MAV_Attitude,  &MavStream, proto_arg, (ComProto_Callback)TaskFrameCTL_MavMsg_Trans);
            SrvComProto.mav_msg_stream(&RadioProto_MAV_RcChannel, &MavStream, proto_arg, (ComProto_Callback)TaskFrameCTL_MavMsg_Trans);
            SrvComProto.mav_msg_stream(&RadioProto_MAV_Altitude,  &MavStream, proto_arg, (ComProto_Callback)TaskFrameCTL_MavMsg_Trans);
            
            /* Proto mavlink message through default port */
            proto_monitor.port_type = Port_USB;
            proto_monitor.port_addr = USB_VCP_Addr;
            // SrvComProto.mav_msg_stream(&TaskProto_MAV_RawIMU,    &MavStream, proto_arg, (ComProto_Callback)TaskFrameCTL_MavMsg_Trans);
            SrvComProto.mav_msg_stream(&TaskProto_MAV_ScaledIMU, &MavStream, proto_arg, (ComProto_Callback)TaskFrameCTL_MavMsg_Trans);
            SrvComProto.mav_msg_stream(&TaskProto_MAV_Attitude,  &MavStream, proto_arg, (ComProto_Callback)TaskFrameCTL_MavMsg_Trans);
            SrvComProto.mav_msg_stream(&TaskProto_MAV_RcChannel, &MavStream, proto_arg, (ComProto_Callback)TaskFrameCTL_MavMsg_Trans);
            // SrvComProto.mav_msg_stream(&TaskProto_MAV_Altitude,  &MavStream, proto_arg, (ComProto_Callback)TaskFrameCTL_MavMsg_Trans);
            SrvComProto.mav_msg_stream(&TaskProto_MAV_Exp_Attitude, &MavStream, proto_arg, (ComProto_Callback)TaskFrameCTL_MavMsg_Trans); 
        }
        else if(tunning_state && (arm_state == DRONE_ARM))
        {
            /* proto tunning parameter */
        }
    }
}

/* still in developping */
static void TaskFrameCTL_CLI_Proc(void)
{
    uint16_t rx_stream_size = 0;
    Shell *shell_obj = Shell_GetInstence();

    /* check CLI stream */
    if(shell_obj && CLI_Monitor.p_rx_stream->p_buf && CLI_Monitor.p_rx_stream->size)
    {
        rx_stream_size = CLI_Monitor.p_rx_stream->size;

        for(uint16_t i = 0; i < rx_stream_size; i++)
        {
            shellHandler(shell_obj, CLI_Monitor.p_rx_stream->p_buf[i]);
            CLI_Monitor.p_rx_stream->p_buf[i] = 0;
            CLI_Monitor.p_rx_stream->size --;
        }
    }
}

static void TaskFrameCTL_MavMsg_Trans(FrameCTL_Monitor_TypeDef *Obj, uint8_t *p_data, uint16_t size)
{
    if(Obj && (Obj->frame_type == ComFrame_MavMsg) && Obj->port_addr && p_data && size)
    {
        switch((uint8_t)(Obj->port_type))
        {
            case Port_Uart:
                TaskFrameCTL_Port_Tx(Obj->port_addr, p_data, size);
                break;

            case Port_USB:
                TaskFrameCTL_DefaultPort_Trans(p_data, size);
                break;

            default:
                return;
        }
    }
}

static void TaskFrameCTL_ConnectStateCheck(void)
{
    uint32_t tunning_time_stamp = 0;
    uint32_t configrator_time_stamp = 0;
    uint32_t tunning_port = 0;
    bool tunning_state = false;
    bool configrator_state = false;
    uint32_t cur_time = SrvOsCommon.get_os_ms();

    /* check configrator and tunning mode time out */
    SrvDataHub.get_configrator_attach_state(&configrator_time_stamp, &configrator_state);
    SrvDataHub.get_tunning_state(&tunning_time_stamp, &tunning_state, &tunning_port);

    if(configrator_state && ((cur_time - configrator_time_stamp) >= CONFIGRATOR_ATTACH_TIMEOUT))
    {
        configrator_state = false;
        configrator_time_stamp = 0;

        SrvOsCommon.enter_critical();
        SrvDataHub.set_configrator_state(configrator_time_stamp, configrator_state);
        SrvOsCommon.exit_critical();
    }

    if(tunning_state && ((cur_time - tunning_time_stamp) >= TUNNING_TIMEOUT))
    {
        tunning_state = false;
        tunning_time_stamp = 0;
        tunning_port = 0;

        SrvOsCommon.enter_critical();
        SrvDataHub.set_tunning_state(tunning_time_stamp, tunning_state, tunning_port);
        SrvOsCommon.exit_critical();
    }
}

/***************************************** CLI Section ***********************************************/
static void TaskFrameCTL_CLI_Trans(uint8_t *p_data, uint16_t size)
{
    if(p_data && size)
    {
        switch ((uint8_t) CLI_Monitor.type)
        {
            case Port_Uart:
                TaskFrameCTL_Port_Tx(CLI_Monitor.port_addr, p_data, size);
                break;
            
            case Port_USB:
                TaskFrameCTL_DefaultPort_Trans(p_data, size);
                break; 

            default:
                break;
        }
    }
}

static void TaskFermeCTL_CLI_EnableControl(uint8_t state)
{
    bool cli_state = state;
    Shell *shell_obj = Shell_GetInstence();

    SrvOsCommon.enter_critical();
    if(state == 0)
    {
        SrvDataHub.set_cli_state(false);
    }
    else
        SrvDataHub.set_cli_state(true);
    SrvOsCommon.exit_critical();
    
    shellPrint(shell_obj, "\r\n\r\n");
    if(state)
    {
        shellPrint(shell_obj, "CLI Enabled\r\n");
    }
    else
    {
        shellPrint(shell_obj, "CLI Disabled\r\n");
        CLI_Monitor.port_addr = 0;
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, CLI_Enable,  TaskFermeCTL_CLI_EnableControl, CLI Enable Control);



