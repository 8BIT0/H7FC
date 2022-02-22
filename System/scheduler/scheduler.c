#include "scheduler.h"
#include "kernel.h"
#include "runtime.h"
#include "linked_list.h"

// coder: 8_B!T0
// bref:
// estabishment a task running system with priority calling functional
// alway calling the highest priority function in all function under the ready table
// reference to ucos (version earler than v8.6)

/*
*******************************   TASK Handle    *******************************

             task0   task1   task2   task3   task4   task5   task6   task7
group0     |_______|_______|_______|_______|_______|_______|_______|_______|
group1     |_______|_______|_______|_______|_______|_______|_______|_______|
group2     |_______|_______|_______|_______|_______|_______|_______|_______|
group3     |_______|_______|_______|_______|_______|_______|_______|_______|
group4     |_______|_______|_______|_______|_______|_______|_______|_______|
group5     |_______|_______|_______|_______|_______|_______|_______|_______|
group6     |_______|_______|_______|_______|_______|_______|_______|_______|
group7     |_______|_______|_______|_______|_______|_______|_______|_______|

*/

#define GET_TASKGROUP_PRIORITY(x) x >> 3
#define GET_TASKINGROUP_PRIORITY(y) y & 0X07

static const uint8_t Task_Priority_List[256] =
    {0, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,  // 0x00 ~ 0x0F
     4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,  // 0x10 ~ 0x1F
     5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,  // 0x20 ~ 0x2F
     4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,  // 0x30 ~ 0x3F
     6, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,  // 0x40 ~ 0x4F
     4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,  // 0x50 ~ 0x5F
     5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,  // 0x60 ~ 0x6F
     4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,  // 0x70 ~ 0x7F
     7, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,  // 0x80 ~ 0x8F
     4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,  // 0x90 ~ 0x9F
     5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,  // 0xA0 ~ 0xAF
     4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,  // 0xB0 ~ 0xBF
     6, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,  // 0xC0 ~ 0xCF
     4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,  // 0xD0 ~ 0xDF
     5, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0,  // 0xE0 ~ 0xEF
     4, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0}; // 0xF0 ~ 0xFF

static Task *TaskMap[Task_Group_Sum][Task_Priority_Sum];
static volatile Task *CurRunTsk_Ptr = NULL;
static volatile Task *NxtRunTsk_Ptr = NULL;

void Os_Init(uint32_t TickFRQ)
{
    Kernel_Init();
    Runtime_Config(TickFRQ);

    for (uint8_t g = Task_Group_0; g < Task_Group_Sum; g++)
    {
        for (uint8_t t = Task_Priority_0; t < Task_Priority_Sum; t++)
        {
            TaskMap[g][t] = NULL;
        }
    }

    // Runtime_SetCallback(RtCallback_Type_Tick, test_pin_ctl);
}

void Os_Start(void)
{
    Runtime_Start();
}

Task_Handle Os_CreateTask(char *name, uint32_t frq)
{
    Task_Handle Tmp_Hdl;

    return Tmp_Hdl;
}

void Os_SchedulerRun(void)
{
}
