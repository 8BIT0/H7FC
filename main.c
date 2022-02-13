#include "kernel.h"
#include "IO_Definition.h"
#include "runtime.h"
#include "Dev_Led.h"

DevLedObj_TypeDef Led1 = {
    .port = LED1_PORT,
    .pin = LED1_PIN,
    // .LedClk_Enable = LED1_CLK_ENABLE,
};

void main(void)
{
    volatile SYSTEM_RunTime Rt = 0;
    volatile SYSTEM_RunTime Lst_Rt = 0;
    bool led_state = false;

    Kernel_Init();

    Runtime_Config(RUNTIME_TICK_FRQ_20K);
    Runtime_Start();

    DevLED.init(Led1);

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
