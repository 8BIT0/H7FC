#ifndef __TASK_PROTOCOL_H
#define __TASK_PROTOCOL_H

#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "Srv_OsCommon.h"
#include "Srv_ComProto.h"
#include "semphr.h"
#include "Bsp_USB.h"
#include "Bsp_Uart.h"
#include "shell_port.h"

#define FrameCTL_Port_Tx_TimeOut 5      /* unit: ms */
#define FrameCTL_MAX_Period 5           /* unit: ms */

#define SLIENT_TIMEOUT 200              /* unit: ms 200ms */
#define TUNNING_TIMEOUT 2000            /* unit: ms 1S */

#define CLI_FUNC_BUF_SIZE 512
#define RADIO_BUFF_SIZE 512
#define RADIO_PORT_BAUD 460800

typedef SrvComProto_ProtoData_Type_List FrameType_List;

typedef enum
{
    Port_USB = 0,
    Port_Uart,
} FrameCTL_PortType_List;

typedef enum
{
    Port_Bypass_None = 0,
    Port_Bypass_RxOnly,
    Port_Bypass_TxOnly,
    Port_Bypass_BothDir,
} Bypass_TypeList;

typedef enum
{
    Slient_Disable = 0,
    Slient_Wait_TimeOut,
    Slient_Wait_Signal,
} Slient_TypeList;

typedef struct
{
    bool enable;
    Bypass_TypeList mode;
    uint8_t *bypass_src;
} Port_Bypass_TypeDef;

typedef struct
{
    FrameCTL_PortType_List type;
    uint8_t port_index;
    uint32_t PortObj_addr;
    uint32_t time_stamp;
} FrameCTL_PortProtoObj_TypeDef;

typedef struct
{
    FrameType_List frame_type;
    FrameCTL_PortType_List port_type;
    uint32_t port_addr;
} FrameCTL_Monitor_TypeDef;

typedef struct
{
    bool init_state;
    
    FrameCTL_PortProtoObj_TypeDef RecObj;
    
    osSemaphoreId p_tx_semphr;
    uint32_t tx_semphr_rls_err;

    BspUSB_VCP_TxStatistic_TypeDef tx_statistic;
    Port_Bypass_TypeDef ByPass_Mode;
} FrameCTL_VCPPortMonitor_TypeDef;

typedef struct
{
    bool init_state;
    FrameCTL_PortProtoObj_TypeDef RecObj;
    Port_Bypass_TypeDef ByPass_Mode;
    
    osSemaphoreId p_tx_semphr;
    uint32_t tx_semphr_rls_err;

    BspUARTObj_TypeDef *Obj;
} FrameCTL_UartPortMonitor_TypeDef;

typedef struct
{
    float exp_roll;
    float exp_pitch;
    float exp_heading;
    float exp_gyrz;
} FrameCTL_MavATTIn_TypeDef;

typedef struct
{
    float exp_x;
    float exp_y;
    float exp_z;
} FrameCTL_MavGyrIn_TypeDef;

typedef struct
{
    float alt;
    float pa;
} FrameCTL_MavAltIn_TypeDef;

typedef struct
{
    float p;
    float p_diff_max;
    float p_diff_min;

    float i;
    float i_max;
    float i_min;

    float d;
} FrameCTL_MavPIDIn_TypeDef;

typedef struct
{
    bool init;
    bool vcp_connect_state;
    uint8_t uart_port_num;

    void *Cur_Tuning_Port;

    FrameCTL_VCPPortMonitor_TypeDef VCP_Port;
    FrameCTL_UartPortMonitor_TypeDef *Uart_Port;
} FrameCTL_PortMonitor_TypeDef;

typedef struct
{
    FrameCTL_PortType_List type;
    uint32_t port_addr;
    SrvComProto_Stream_TypeDef *p_rx_stream;
    SrvComProto_Stream_TypeDef *p_proc_stream;
    
    Slient_TypeList slient_type;
    uint16_t slient_timeout;
} FrameCTL_CLIMonitor_TypeDef;

void TaskFrameCTL_Init(uint32_t period);
void TaskFrameCTL_Core(void *arg);

#endif
