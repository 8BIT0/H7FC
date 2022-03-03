#include "kernel.h"
#include "IO_Definition.h"
#include "runtime.h"
#include "Dev_Led.h"
#include "system_cfg.h"
#include "debug_util.h"
#include "scheduler.h"

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
    Os_Start();

    while (true)
    {
    }
}

void Run(void)
{
    volatile SYSTEM_RunTime Rt = 0;
    volatile SYSTEM_RunTime Lst_Rt = 0;
    bool led_state = false;

    DevLED.init(Led1);
    DebugPin.init(Debug_PC0);

    Kernel_Init();

    Runtime_Config(RUNTIME_TICK_FRQ_40K);
    Runtime_Start();

    // Runtime_SetCallback(RtCallback_Type_Tick, test_pin_ctl);

    while (1)
    {
        Rt = Get_CurrentRunningMs();

        if ((Rt % 50 == 0) && (Lst_Rt != Rt))
        {
            led_state = !led_state;
            Lst_Rt = Rt;
        }

        DevLED.ctl(Led1, led_state);
    }
}