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

static FrameCTL_Monitor_TypeDef FrameCTL_Monitor;
static bool FrameCTL_MavProto_Enable = false;
static FrameCTL_PortMonitor_TypeDef PortMonitor = {.init = false};
static uint32_t FrameCTL_Period = 0;
static uint8_t MavShareBuf[1024];
static uint32_t Radio_Addr = 0;
static uint32_t USB_VCP_Addr = 0;

SrvComProto_Stream_TypeDef MavStream = {
    .p_buf = MavShareBuf,
    .size = 0,
    .max_size = sizeof(MavShareBuf),
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

void TaskFrameCTL_Init(uint32_t period)
{
    FrameCTL_Period = FrameCTL_MAX_Period;

    /* USB VCP as defaut port to tune parameter and frame porotcol */
    memset(&PortMonitor, 0, sizeof(PortMonitor));

    TaskFrameCTL_DefaultPort_Init(&PortMonitor);
    TaskFrameCTL_RadioPort_Init(&PortMonitor);
    Radio_Addr = TaskFrameCTL_Set_RadioPort(Port_Uart, 0);

    PortMonitor.init = true;
    
    /* init radio protocol*/
    FrameCTL_MavProto_Enable = TaskFrameCTL_MAV_Msg_Init();

    if(period && (period <= FrameCTL_MAX_Period))
    {
        FrameCTL_Period = period;
    }
}

void TaskFrameCTL_Core(void *arg)
{
    uint32_t per_time = SrvOsCommon.get_os_ms();

    while(1)
    {
        /* frame protocol process */
        TaskFrameCTL_PortFrameOut_Process();
        TaskFrameCTL_ConnectStateCheck();

        SrvOsCommon.precise_delay(&per_time, FrameCTL_Period);
    }
}

/*************************** ByPass Mode is still on developping *********************************/
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
                    BspUart.set_rx_callback(&(monitor->Uart_Port[i].Obj), TaskFrameCTL_Port_Rx_Callback);
                    BspUart.set_tx_callback(&(monitor->Uart_Port[i].Obj), TaskFrameCTL_Port_TxCplt_Callback);

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
    if(obj_addr && p_data && size)
    {
        BspUart.send();
    }
}

static void TaskFrameCTL_Port_Rx_Callback(uint32_t RecObj_addr, uint8_t *p_data, uint16_t size)
{
    SrvComProto_Msg_StreamIn_TypeDef stream_in;
    FrameCTL_PortProtoObj_TypeDef *p_RecObj = NULL;

    /* use mavlink protocol tuning the flight parameter */
    if(p_data && size && RecObj_addr)
    {
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
    
        if(stream_in.valid)
        {
            /* tag on recive time stamp */
            /* first come first serve */
            /* in case two different port tuning the same function or same parameter at the same time */
            /* if attach to configrator or in tunning then lock moto */
            if(stream_in.pac_type == ComFrame_MavMsg)
            {
                /* check mavline message frame type */
            }
        }
    }
}

static void TaskFrameCTL_Port_TxCplt_Callback(uint32_t RecObj_addr, uint8_t *p_data, uint32_t *size)
{
    UNUSED(p_data);
    UNUSED(size);

    FrameCTL_PortProtoObj_TypeDef *p_RecObj = NULL;
    FrameCTL_UartPortMonitor_TypeDef *p_UartPortObj = NULL;
    FrameCTL_VCPPortMonitor_TypeDef *p_USBPortObj = NULL;
    osSemaphoreId semID = NULL;
    uint32_t *p_rls_err_cnt = NULL;

    if(RecObj_addr)
    {
        p_RecObj = RecObj_addr;

        if(p_RecObj->PortObj_addr)
        {
            switch((uint8_t) p_RecObj->type)
            {
                case Port_USB:
                    p_USBPortObj = (FrameCTL_VCPPortMonitor_TypeDef *)(p_RecObj->PortObj_addr);

                    if(p_USBPortObj->init_state && p_USBPortObj->p_tx_semphr)
                    {
                        semID = p_USBPortObj->p_tx_semphr;
                        p_rls_err_cnt = &p_USBPortObj->tx_semphr_rls_err;
                    }
                    break;

                case Port_Uart:
                    p_UartPortObj = (FrameCTL_UartPortMonitor_TypeDef *)(p_RecObj->PortObj_addr);

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

static void TaskFrameCTL_PortFrameOut_Process(void)
{
    FrameCTL_Monitor_TypeDef proto_monitor;
    bool tunning_state = false;
    uint32_t tunning_time_stamp = 0;
    uint32_t tunning_port = 0;

    proto_monitor.frame_type = ComFrame_MavMsg;

    if(FrameCTL_MavProto_Enable && PortMonitor.VCP_Port.init_state)
    {
        /* when attach to configrator then disable radio port trans use default port trans mav data */
        /* check other port init state */

        /* if in tunning than halt general frame protocol */
        SrvDataHub.get_tunning_state(&tunning_time_stamp, &tunning_state, &tunning_port);

        if(!tunning_state)
        {
            /* Proto mavlink message through Radio */
            // proto_monitor.port_type = Port_Uart;
            // proto_monitor.port_addr = Radio_Addr;
            // SrvComProto.mav_msg_stream(&TaskProto_MAV_RawIMU,    &MavStream, , (ComProto_Callback)TaskFrameCTL_MavMsg_Trans);
            // SrvComProto.mav_msg_stream(&TaskProto_MAV_ScaledIMU, &MavStream, , (ComProto_Callback)TaskFrameCTL_MavMsg_Trans);
            // SrvComProto.mav_msg_stream(&TaskProto_MAV_Attitude,  &MavStream, , (ComProto_Callback)TaskFrameCTL_MavMsg_Trans);
            // SrvComProto.mav_msg_stream(&TaskProto_MAV_RcChannel, &MavStream, , (ComProto_Callback)TaskFrameCTL_MavMsg_Trans);
            
            /* Proto mavlink message through default port */
            // proto_monitor.port_type = Port_USB;
            // proto_monitor.port_addr = USB_VCP_Addr;
            // SrvComProto.mav_msg_stream(&TaskProto_MAV_RawIMU,    &MavStream, , (ComProto_Callback)TaskFrameCTL_MavMsg_Trans);
            // SrvComProto.mav_msg_stream(&TaskProto_MAV_ScaledIMU, &MavStream, , (ComProto_Callback)TaskFrameCTL_MavMsg_Trans);
            // SrvComProto.mav_msg_stream(&TaskProto_MAV_Attitude,  &MavStream, , (ComProto_Callback)TaskFrameCTL_MavMsg_Trans);
            // SrvComProto.mav_msg_stream(&TaskProto_MAV_RcChannel, &MavStream, , (ComProto_Callback)TaskFrameCTL_MavMsg_Trans);
        }
        else
        {
            /* proto tunning parameter */
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
                break;

            case Port_USB:
                if(PortMonitor.VCP_Port.init_state)
                {
                    TaskFrameCTL_DefaultPort_Trans(p_data, size);
                }
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
