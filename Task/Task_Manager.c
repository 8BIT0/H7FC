#include "Task_Manager.h"
#include "Task_Log.h"
#include "Task_Protocol.h"
#include "Task_Control.h"
#include "Task_SensorInertial.h"
#include "Task_Telemetry.h"
#include "scheduler.h"
#include "debug_util.h"
#include "IO_Definition.h"
#include "runtime.h"
#include "Dev_Led.h"
#include "DiskIO.h"
#include "DataPipe/DataPipe.h"

Task_Handle TaskProtocol_Handle = NULL;
Task_Handle TaskInertial_Handle = NULL;
Task_Handle TaskControl_Handle = NULL;
Task_Handle TaskLog_Handle = NULL;
Task_Handle TestTelemetry_Handle = NULL;

void Test2(Task_Handle handle);

void test_PC0_ctl(void)
{
    DebugPin.ctl(Debug_PC0, true);
    DebugPin.ctl(Debug_PC0, false);
}

void test_PC1_ctl(void)
{
    DebugPin.ctl(Debug_PC1, true);
    DebugPin.ctl(Debug_PC1, false);
}

void test_PC2_ctl(void)
{
    DebugPin.ctl(Debug_PC2, true);
    DebugPin.ctl(Debug_PC2, false);
}

void Task_Manager_Init(void)
{
    DevLED.init(Led1);
    DevLED.init(Led2);
    DevLED.init(Led3);

    DebugPin.init(Debug_PC0);
    DebugPin.init(Debug_PC1);
    DebugPin.init(Debug_PC2);
    DebugPin.init(Debug_PC3);
    DebugPin.init(Debug_PB3);
    DebugPin.init(Debug_PB4);
    DebugPin.init(Debug_PB5);
    DebugPin.init(Debug_PB6);
    DebugPin.init(Debug_PB10);

    /* ESC port init */

    /* vol ADC init */

    /* cur ADC init */

    DataPipe_Init();

    TaskProtocol_Init();
    TaskInertial_Init();
    TaskTelemetry_Init();
    TaskControl_Init();
    TaskLog_Init();
}

void Task_Manager_CreateTask(void)
{
    TaskInertial_Handle = Os_CreateTask("Inertial Sample", TASK_EXEC_2KHZ, Task_Group_0, Task_Priority_0, TaskInertical_Core, 1024);
    TaskControl_Handle = Os_CreateTask("Control", TASK_EXEC_1KHZ, Task_Group_0, Task_Priority_1, TaskControl_Core, 1024);
    TaskProtocol_Handle = Os_CreateTask("Protocl", TASK_EXEC_100HZ, Task_Group_1, Task_Priority_0, TaskProtocol_Core, 1024);
    TaskLog_Handle = Os_CreateTask("Data Log", TASK_EXEC_200HZ, Task_Group_2, Task_Priority_0, TaskLog_Core, 1024);
    TestTelemetry_Handle = Os_CreateTask("Telemetry", TASK_EXEC_100HZ, Task_Group_0, Task_Priority_2, TaskTelemetry_Core, 512);
}
