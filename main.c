#include "kernel.h"
#include "IO_Definition.h"
#include "runtime.h"
#include "Dev_Led.h"
#include "system_cfg.h"
#include "debug_util.h"
#include "scheduler.h"

Task_Handle Blink_Task = NULL;
void Run(Task_Handle hdl);

DevLedObj_TypeDef Led1 = {
    .port = LED1_PORT,
    .pin = LED1_PIN,
    .default_state = true,
};

DebugPinObj_TypeDef Debug_PC0 = {
    .port = GPIOC,
    .pin = GPIO_PIN_0,
    .default_state = false,
};

void test_pin_ctl(void)
{
    DebugPin.ctl(Debug_PC0, true);
    DebugPin.ctl(Debug_PC0, false);
}

extern uint32_t msp;

void main(void)
{
    Os_Init(RUNTIME_TICK_FRQ_40K);

    /* create task down below */
    Blink_Task = Os_CreateTask("Blink", TASK_EXEC_10KHZ, Task_Group_0, Task_Group_0, Run, 256);
    /* create task up top */

    Os_Start();
}

void Run(Task_Handle handle)
{
    volatile SYSTEM_RunTime Rt = 0;
    static volatile SYSTEM_RunTime Lst_Rt = 0;
    static bool led_state = false;

    DevLED.init(Led1);
    DebugPin.init(Debug_PC0);

    Rt = Get_CurrentRunningMs();

    if ((Rt % 50 == 0) && (Lst_Rt != Rt))
    {
        led_state = !led_state;
        Lst_Rt = Rt;
    }

    DevLED.ctl(Led1, led_state);
    test_pin_ctl();
}