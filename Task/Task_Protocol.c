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
#include "mmu.h"
/* test code */
#include "DiskIO.h"
/* test code */

static bool test = false;

#define VCP_QUEUE_BUFF_SIZE 4096

/* internal vriable */
static QueueObj_TypeDef VCP_ProtoQueue; /* Send Queue */
static bool VCP_Queue_CreateState = false;
static ProtoQueue_State_List VCPQueue_State = ProtoQueue_Idle;
static bool VCP_Connect_State = false; /* USB connect state */

/* task state var */
static TaskProto_State_List task_state = TaskProto_Core;
static bool Shell_Mode = false;

/* internal function */
static void TaskProtocol_Main(uint8_t *data, uint16_t size);
static bool TaskProtocol_TransBuff(uint8_t *data, uint16_t size);
static void TaskProtocol_Rec(uint8_t *data, uint16_t len);
static void TaskProtocol_PlugDetect_Callback(void);
ProtoQueue_State_List TaskProto_PushProtocolQueue(uint8_t *p_data, uint16_t size);

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

ProtoQueue_State_List TaskProto_PushProtocolQueue(uint8_t *p_data, uint16_t size)
{
    /* push into send queue */
    if (VCP_Queue_CreateState &&
        ((VCPQueue_State == ProtoQueue_Ok) ||
         (VCPQueue_State == ProtoQueue_Idle)) &&
        ((Queue.state(VCP_ProtoQueue) == Queue_empty) ||
         (Queue.state(VCP_ProtoQueue) == Queue_ok)))
    {
        VCPQueue_State = ProtoQueue_Busy;

        /* push send data into VCP queue */
        Queue.push(&VCP_ProtoQueue, p_data, size);

        VCPQueue_State = ProtoQueue_Ok;

        return ProtoQueue_Ok;
    }
    else if (!VCP_Queue_CreateState)
    {
        VCPQueue_State = ProtoQueeu_Error;
        return ProtoQueeu_Error;
    }
    else if (Queue.state(VCP_ProtoQueue) == Queue_full)
    {
        VCPQueue_State = ProtoQueue_Full;
        return ProtoQueue_Full;
    }
    else if ((VCPQueue_State != ProtoQueue_Ok) ||
             (VCPQueue_State != ProtoQueue_Idle))
    {
        return VCPQueue_State;
    }
}

static bool TaskProtocol_TransBuff(uint8_t *data, uint16_t size)
{
    if (CDC_Transmit_FS(data, size) != USBD_OK)
        return false;

    return true;
}

void TaskProtocol_Core(Task_Handle hdl)
{
    uint8_t *p_buf = NULL;
    uint16_t p_buf_size = 0;

    /* test code */
    static bool led_state = false;
    static uint8_t cnt = 0;
    /* test code */

    DebugPin.ctl(Debug_PC3, true);

    switch ((uint8_t)task_state)
    {
    case TaskProto_Core:
        TaskProtocol_Main(NULL, 0);

        /* check vcp send queue state */
        /* if it has any data then send them out */
        if (test && ((Queue.state(VCP_ProtoQueue) == Queue_ok) || (Queue.state(VCP_ProtoQueue) == Queue_full)) && Queue.size(VCP_ProtoQueue))
        {
            p_buf = (uint8_t *)MMU_Malloc(Queue.size(VCP_ProtoQueue));

            if (p_buf)
            {
                p_buf_size = Queue.size(VCP_ProtoQueue);
                Queue.pop(&VCP_ProtoQueue, p_buf, p_buf_size);

                TaskProtocol_TransBuff(p_buf, p_buf_size);

                MMU_Free(p_buf);
            }
        }

        /* test code */
        if (cnt < 5)
        {
            cnt++;
        }
        else
        {
            cnt = 0;
            led_state = !led_state;

            DevLED.ctl(Led2, led_state);
            FileWrite_Test();
        }
        /* test code */

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
    test = true;
}

static void TaskProtocol_PlugDetect_Callback(void)
{
    static uint8_t a;

    a++;
}

static void shell_test(void)
{
    usb_printf("\t8_B!T0 Shell test\r\n");
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, test, shell_test, Shell Test);
