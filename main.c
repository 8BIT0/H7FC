#include "kernel.h"
#include "Task_Manager.h"

void main(void)
{
    Kernel_Init();

    /* create task down below */
    Task_Manager_Init();
    Task_Manager_CreateTask();
    /* create task up top */
}
