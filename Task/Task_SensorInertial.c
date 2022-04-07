#include "Task_SensorInertial.h"
#include "scheduler.h"
#include "debug_util.h"
#include "runtime.h"
#include "IO_Definition.h"
#include "SrvMPU_Sample.h"
#include "debug_util.h"
#include "error_log.h"

/* internal var */
static Task_SensorInertial_State TaskInertial_State = Task_SensorInertial_Core;
static Error_Handler TaskInertial_ErrorLog_Handle = NULL;

/* internal function */
static void TaskInertical_Blink_Notification(uint16_t duration);

/* external function */

void TaskInertial_Init(void)
{
    /* regist error */

    SrvIMU_Init();
}

void TaskInertical_Core(Task_Handle hdl)
{
    DebugPin.ctl(Debug_PB5, true);
    switch ((uint8_t)TaskInertial_State)
    {
    case Task_SensorInertial_Core:
        TaskInertical_Blink_Notification(250);
        break;

    case Task_SensorInertial_Error:
        break;

    default:
        break;
    }
    DebugPin.ctl(Debug_PB5, false);
}

static void TaskInertical_Blink_Notification(uint16_t duration)
{
    SYSTEM_RunTime Rt = 0;
    static SYSTEM_RunTime Lst_Rt = 0;
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
