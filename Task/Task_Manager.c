#include "Task_Manager.h"
#include "Task_Protocol.h"
#include "scheduler.h"

Task_Handle TaskProtocol_Handle;

void Task_Manager(void)
{
    TaskProtocol_Handle = Os_CreateTask("Protocl", TASK_EXEC_1KHZ, Task_Group_1, Task_Priority_0, TaskProtocol_Core, 512);
}
