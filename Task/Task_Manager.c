#include "Task_Manager.h"
#include "Task_Protocol.h"
#include "SrvMPU_Sample.h"
#include "scheduler.h"

Task_Handle TaskProtocol_Handle;

void Task_Manager(void)
{
    int8_t error = SrvIMU_Init();

    if (error == -4)
    {
        TaskProtocol_GetSrvMPU_InitState(SrvIMU_GetPri_InitError());
    }
    else
        TaskProtocol_GetSrvMPU_InitState(error);

    TaskProtocol_Handle = Os_CreateTask("Protocl", TASK_EXEC_1KHZ, Task_Group_1, Task_Priority_0, TaskProtocol_Core, 512);
}
