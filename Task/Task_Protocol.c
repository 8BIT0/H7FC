#include "Task_Protocol.h"
#include "scheduler.h"
#include "shell.h"
#include "usb_device.h"

/* task state var */
static TaskProto_State_List task_state = TaskProto_Init;

static bool TaskProtocol_Init(void)
{
    return USB_DEVICE_Init();
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
        break;

    case TaskProto_Error_Proc:
        break;

    default:
        break;
    }
}
