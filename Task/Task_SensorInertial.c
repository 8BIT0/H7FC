#include "Task_SensorInertial.h"
#include "scheduler.h"

/* internal var */
static Task_SensorInertial_State TaskInertial_State = Task_SensorInertial_Init;

static bool TaskInertical_Init(void)
{
    return true;
}

void TaskInertical_Core(Task_Handle hdl)
{
    switch ((uint8_t)TaskInertial_State)
    {
    case Task_SensorInertial_Init:
        if (!TaskInertical_Init())
        {
            TaskInertial_State = Task_SensorInertial_Error;
            break;
        }

        TaskInertial_State = Task_SensorInertial_Core;
        break;

    case Task_SensorInertial_Core:
        break;

    case Task_SensorInertial_Error:
        break;

    default:
        break;
    }
}