#include "Task_Protocol.h"
#include "scheduler.h"
#include "shell.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>

/* task state var */
static TaskProto_State_List task_state = TaskProto_Init;
static int8_t SrvIMU_InitState = 0;

/* internal function */
static void TaaskProtocol_Main(uint8_t *data, uint16_t size);
static bool TaskProtocol_TransBuff(uint8_t *data, uint16_t size);

void TaskProtocol_GetSrvMPU_InitState(int8_t state)
{
    SrvIMU_InitState = state;
}

static bool TaskProtocol_Init(void)
{
    return USB_DEVICE_Init();
}

static bool TaskProtocol_TransBuff(uint8_t *data, uint16_t size)
{
    if (CDC_Transmit_FS(data, size) != USBD_OK)
        return false;

    return true;
}

void TaskProtocol_Core(Task_Handle hdl)
{
    switch ((uint8_t)task_state)
    {
    case TaskProto_Init:
        if (!TaskProtocol_Init())
        {
            task_state = TaskProto_Error_Proc;
            break;
        }

        task_state = TaskProto_Core;
        break;

    case TaskProto_Core:
        usb_printf("MPU6000 Init State : %d\r\n", SrvIMU_InitState);

        TaaskProtocol_Main(NULL, 0);
        break;

    case TaskProto_Error_Proc:
        break;

    default:
        break;
    }
}

static void TaaskProtocol_Main(uint8_t *data, uint16_t size)
{
}
