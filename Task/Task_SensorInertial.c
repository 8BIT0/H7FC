#include "Task_SensorInertial.h"
#include "scheduler.h"
#include "debug_util.h"
#include "runtime.h"
#include "IO_Definition.h"
#include "SrvMPU_Sample.h"

/* internal var */
static Task_SensorInertial_State TaskInertial_State = Task_SensorInertial_Core;

void TaskInertial_Init(void)
{
}

void TaskInertical_Core(Task_Handle hdl)
{
    switch ((uint8_t)TaskInertial_State)
    {
    case Task_SensorInertial_Core:
        Blink_Notification(50);
        break;

    case Task_SensorInertial_Error:
        break;

    default:
        break;
    }
}

void Blink_Notification(uint16_t duration)
{
    volatile SYSTEM_RunTime Rt = 0;
    static volatile SYSTEM_RunTime Lst_Rt = 0;
    static bool led_state = false;

    Rt = Get_CurrentRunningMs();

    if ((Rt % duration == 0) && (Lst_Rt != Rt))
    {
        led_state = !led_state;
        Lst_Rt = Rt;
    }

    DevLED.ctl(Led2, led_state);
    // test_PC1_ctl();
}
