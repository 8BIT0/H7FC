#include "Task_Protocol.h"
#include "scheduler.h"
#include "shell.h"
#include "shell_port.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "debug_util.h"
#include <stdio.h>
#include "Dev_MPU6000.h"

/* task state var */
static TaskProto_State_List task_state = TaskProto_Init;
static bool Shell_Mode = false;

/* test */
static int8_t SrvIMU_InitState = 0;
DebugPinObj_TypeDef Debug_PC3 = {
    .port = GPIOC,
    .pin = GPIO_PIN_3,
    .init_state = false,
};

/* internal function */
static void TaaskProtocol_Main(uint8_t *data, uint16_t size);
static bool TaskProtocol_TransBuff(uint8_t *data, uint16_t size);
static void TaskProtocol_Rec(uint8_t *data, uint16_t len);

void TaskProtocol_GetSrvMPU_InitState(int8_t state)
{
    SrvIMU_InitState = state;
}

bool TaskProtocol_Init(void)
{
    DebugPin.init(Debug_PC3);

    if (!USB_DEVICE_Init())
    {
        task_state = TaskProto_Error_Proc;
        return false;
    }

    usb_setrec_callback(TaskProtocol_Rec);
    Shell_Init(TaskProtocol_TransBuff);

    usb_printf("Task_Proto Init\r\n");

    task_state = TaskProto_Core;
    return true;
}

static bool TaskProtocol_TransBuff(uint8_t *data, uint16_t size)
{
    if (CDC_Transmit_FS(data, size) != USBD_OK)
        return false;

    return true;
}

void TaskProtocol_Core(Task_Handle hdl)
{
    DebugPin.ctl(Debug_PC3, true);

    switch ((uint8_t)task_state)
    {
    case TaskProto_Core:
        usb_printf("test \r\n");

        TaaskProtocol_Main(NULL, 0);
        break;

    case TaskProto_Error_Proc:
        break;

    default:
        break;
    }

    DebugPin.ctl(Debug_PC3, false);
}

static void TaaskProtocol_Main(uint8_t *data, uint16_t size)
{
}

static void TaskProtocol_Rec(uint8_t *data, uint16_t len)
{
    // shellHandler(Shell_GetInstence(), data[i]);
    TaskProtocol_TransBuff(data, len);
}

static void shell_test(void)
{
    usb_printf("8_B!T0 Shell test\r\n");
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, test, shell_test, Shell Test);
