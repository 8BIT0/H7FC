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
static bool traverse_start = false;

static volatile TaskMap_TypeDef TskHdl_RdyMap = {.Grp = 0, .TskInGrp[0] = 0, .TskInGrp[1] = 0, .TskInGrp[2] = 0, .TskInGrp[3] = 0, .TskInGrp[4] = 0, .TskInGrp[5] = 0, .TskInGrp[6] = 0, .TskInGrp[7] = 0};
static volatile TaskMap_TypeDef TskHdl_PndMap = {.Grp = 0, .TskInGrp[0] = 0, .TskInGrp[1] = 0, .TskInGrp[2] = 0, .TskInGrp[3] = 0, .TskInGrp[4] = 0, .TskInGrp[5] = 0, .TskInGrp[6] = 0, .TskInGrp[7] = 0};
static volatile TaskMap_TypeDef TskHdl_BlkMap = {.Grp = 0, .TskInGrp[0] = 0, .TskInGrp[1] = 0, .TskInGrp[2] = 0, .TskInGrp[3] = 0, .TskInGrp[4] = 0, .TskInGrp[5] = 0, .TskInGrp[6] = 0, .TskInGrp[7] = 0};

/* internal function */
static void Os_ResetTask_Data(Task *task);
static void Os_Set_TaskReady(Task *tsk);
static void Os_Clr_TaskReady(Task *tsk);
static void Os_SchedulerRun(void);

void Os_Init(uint32_t TickFRQ)
{
    Kernel_Init();
    Runtime_Config(TickFRQ);

    for (uint8_t g = Task_Group_0; g < Task_Group_Sum; g++)
    {
        for (uint8_t t = Task_Priority_0; t < Task_Priority_Sum; t++)
        {
            Os_ResetTask_Data(&TaskMap[g][t]);
            TaskMap[g][t] = NULL;
        }
    }
}

void Os_Start(void)
{
    Runtime_Start();
    // Runtime_SetCallback(RtCallback_Type_Tick, test_pin_ctl);
}

Task_Handle Os_CreateTask(char *name, uint32_t frq)
{
    Task_Handle Tmp_Hdl;

    return Tmp_Hdl;
}

static void Os_ResetTask_Data(Task *task)
{
    task->priority = 0;
    task->Task_name = NULL;
    task->exec_frq = 0;
    task->exec_interval_us = 0;
    task->Exec_Func = NULL;

    task->Exec_status.detect_exec_frq = 0;
    task->Exec_status.detect_exec_time_arv = 0;
    task->Exec_status.detect_exec_time_max = 0;
    task->Exec_status.Exec_Times = 0;
    task->Exec_status.cpu_opy = 0;
    task->Exec_status.totlal_running_time = 0;

    task->delay_info.on_delay = false;
    task->delay_info.tsk_hdl = 0;
    task->delay_info.time_unit = 0;

    List_ItemInit(&task->delay_item, &task->delay_info);

    RuntimeObj_Reset(&(task->Exec_Time));

    task->Exec_status.State = Task_Init;
}

static void Os_Set_TaskReady(Task *tsk)
{
    uint8_t grp_id = GET_TASKGROUP_PRIORITY(tsk->priority);
    uint8_t tsk_id = GET_TASKINGROUP_PRIORITY(tsk->priority);

    // set current group flag to ready
    TskHdl_RdyMap.Grp.Flg |= 1 << grp_id;
    // set current task under this group flag to ready
    TskHdl_RdyMap.TskInGrp[grp_id].Flg |= 1 << tsk_id;

    tsk->Exec_status.State = Task_Ready;
}

static void Os_Clr_TaskReady(Task *tsk)
{
    uint8_t grp_id = GET_TASKGROUP_PRIORITY(tsk->priority);
    uint8_t tsk_id = GET_TASKINGROUP_PRIORITY(tsk->priority);

    TskHdl_RdyMap.TskInGrp[grp_id].Flg &= ~(1 << tsk_id);
    if (TskHdl_RdyMap.TskInGrp[grp_id].Flg == 0)
    {
        TskHdl_RdyMap.Grp.Flg &= ~(1 << grp_id);
    }
}

static void Os_SchedulerRun(void)
{
}
