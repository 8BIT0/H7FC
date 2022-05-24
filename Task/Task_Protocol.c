/*
 *  coder: 8_B!T0
 *  bref: use this task make FC communicate to computer configuration
 */
#include "Task_Protocol.h"
#include "scheduler.h"
#include "shell.h"
#include "shell_port.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "debug_util.h"
#include <stdio.h>
#include "Dev_MPU6000.h"
#include "queue.h"
#include "Dev_Led.h"
#include "IO_Definition.h"
#include "Bsp_GPIO.h"
#include "error_log.h"

#define VCP_QUEUE_BUFF_SIZE 4096

/* internal vriable */
static QueueObj_TypeDef VCP_ProtoQueue; /* Send Queue */
static bool VCP_Queue_CreateState = false;
static bool VCP_Connect_State = false; /* USB connect state */

/* task state var */
static TaskProto_State_List task_state = TaskProto_Core;
static bool Shell_Mode = false;

/* internal function */
static void TaskProtocol_Main(uint8_t *data, uint16_t size);
static bool TaskProtocol_TransBuff(uint8_t *data, uint16_t size);
static void TaskProtocol_Rec(uint8_t *data, uint16_t len);
static void TaskProtocol_PlugDetect_Callback(void);
static ProtoQueue_State_List TaskProto_PushProtocolQueue(uint8_t *p_data, uint16_t size);

bool TaskProtocol_Init(void)
{
    if (!USB_DEVICE_Init())
    {
        task_state = TaskProto_Error_Proc;
        return false;
    }

    /* init USB connect detect pin */
    // BspGPIO.in_init(USB_DctPin);

    if (!Queue.create(&VCP_ProtoQueue, "VCP Send Queue", VCP_QUEUE_BUFF_SIZE))
        return false;

    VCP_Queue_CreateState = true;

    usb_setrec_callback(TaskProtocol_Rec);
    Shell_Init(TaskProtocol_TransBuff);

    ErrorLog.set_callback(Error_Out_Callback, TaskProto_PushProtocolQueue);

    task_state = TaskProto_Core;
    return true;
}

static ProtoQueue_State_List TaskProto_PushProtocolQueue(uint8_t *p_data, uint16_t size)
{
    /* push into send queue */
    if (VCP_Queue_CreateState && ((Queue.state(VCP_ProtoQueue) == Queue_Full) || (Queue.state(VCP_ProtoQueue) == Queue_ok)))
    {
        /* push send data into VCP queue */
    }

    return ProtoQueeu_Error;
}

static bool TaskProtocol_TransBuff(uint8_t *data, uint16_t size)
{
    if (CDC_Transmit_FS(data, size) != USBD_OK)
        return false;

    return true;
}

void TaskProtocol_Core(Task_Handle hdl)
{
    static uint32_t a, b = 0;

    DebugPin.ctl(Debug_PC3, true);

    switch ((uint8_t)task_state)
    {
    case TaskProto_Core:
        TaskProtocol_Main(NULL, 0);

        /* check vcp send queue state */
        /* if it has any data then send them out */
        if ((Queue.state() == Queue_ok) || (Queue.state() == Queue_full))
        {
            /* send */
        }
        break;

    case TaskProto_Error_Proc:
        break;

    default:
        break;
    }

    DebugPin.ctl(Debug_PC3, false);
}

static void TaskProtocol_Main(uint8_t *data, uint16_t size)
{
}

static void TaskProtocol_Rec(uint8_t *data, uint16_t len)
{
    // shellHandler(Shell_GetInstence(), data[i]);
    // TaskProtocol_TransBuff(data, len);
}

static void TaskProtocol_PlugDetect_Callback(void)
{
    static uint8_t a;

    a++;
}

static void shell_test(void)
{
    usb_printf("8_B!T0 Shell test\r\n");
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, test, shell_test, Shell Test);
