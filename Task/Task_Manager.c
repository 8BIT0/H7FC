#include "Task_Manager.h"
#include "Task_Protocol.h"
#include "Task_SensorInertial.h"
#include "scheduler.h"
#include "debug_util.h"
#include "IO_Definition.h"
#include "runtime.h"
#include "Dev_Led.h"

Task_Handle TaskProtocol_Handle = NULL;
Task_Handle TaskInertial_Handle = NULL;
Task_Handle Test_Task = NULL;
Task_Handle Test2_Task = NULL;

void Run(Task_Handle handle);
void Test(Task_Handle handle);
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

    TaskInertial_Init();
    TaskProtocol_Init();
}

void Task_Manager_CreateTask(void)
{
    TaskInertial_Handle = Os_CreateTask("Inertial Sample", TASK_EXEC_2KHZ, Task_Group_0, Task_Group_0, TaskInertical_Core, 2048);
    TaskProtocol_Handle = Os_CreateTask("Protocl", TASK_EXEC_20HZ, Task_Group_1, Task_Priority_0, TaskProtocol_Core, 1024);
    Test2_Task = Os_CreateTask("test2", TASK_EXEC_1KHZ, Task_Group_0, Task_Group_2, Test2, 256);
    // Test_Task = Os_CreateTask("test delay", TASK_EXEC_2KHZ, Task_Group_0, Task_Group_1, Test, 256);
}

void Test2(Task_Handle handle)
{
    SYSTEM_RunTime Rt = 0;
    static SYSTEM_RunTime Lst_Rt = 0;
    static bool led_state = false;

    DebugPin.ctl(Debug_PB4, true);
    DebugPin.ctl(Debug_PB4, false);

    Rt = Get_CurrentRunningMs();

    if ((Rt % 100 == 0) && (Lst_Rt != Rt))
    {
        led_state = !led_state;
        Lst_Rt = Rt;
    }

    DevLED.ctl(Led1, led_state);
    // DevLED.ctl(Led3, led_state);
}

void Test(Task_Handle handle)
{
    // DebugPin.ctl(Debug_PC0, true);
    // DebugPin.ctl(Debug_PB3, true);
    // Os_TaskDelay_Ms(handle, 10);

    // DebugPin.ctl(Debug_PC0, false);
    // DebugPin.ctl(Debug_PB3, false);
    // Os_TaskDelay_Ms(handle, 20);

    // DebugPin.ctl(Debug_PC0, true);
    // DebugPin.ctl(Debug_PB3, true);
    // Os_TaskDelay_Ms(handle, 30);

    // DebugPin.ctl(Debug_PC0, false);
    // DebugPin.ctl(Debug_PB3, false);
    // Os_TaskDelay_Ms(handle, 40);

    // DebugPin.ctl(Debug_PB3, true);
    // DebugPin.ctl(Debug_PB3, false);
}
