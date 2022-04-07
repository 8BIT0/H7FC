#include "kernel.h"
#include "system_cfg.h"
#include "scheduler.h"
#include "Task_Manager.h"

void main(void)
{
    // runtime module ticked frequence by 8K can make system stablize while using 8M External Oscillator
    Os_Init(RUNTIME_TICK_FRQ_8K);

    /* create task down below */
    Task_Manager_Init();
    Task_Manager_CreateTask();
    /* create task up top */

    Os_Start();
}
