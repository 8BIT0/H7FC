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

/* internal variable */
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

static Task *TaskPtr_Map[Task_Group_Sum][Task_Priority_Sum];
static volatile Task *CurRunTsk_Ptr = NULL;
static volatile Task *NxtRunTsk_Ptr = NULL;
static bool traverse_start = false;

static volatile TaskMap_TypeDef TskHdl_RdyMap = {.Grp = 0, .TskInGrp[0] = 0, .TskInGrp[1] = 0, .TskInGrp[2] = 0, .TskInGrp[3] = 0, .TskInGrp[4] = 0, .TskInGrp[5] = 0, .TskInGrp[6] = 0, .TskInGrp[7] = 0};
static volatile TaskMap_TypeDef TskHdl_PndMap = {.Grp = 0, .TskInGrp[0] = 0, .TskInGrp[1] = 0, .TskInGrp[2] = 0, .TskInGrp[3] = 0, .TskInGrp[4] = 0, .TskInGrp[5] = 0, .TskInGrp[6] = 0, .TskInGrp[7] = 0};
static volatile TaskMap_TypeDef TskHdl_BlkMap = {.Grp = 0, .TskInGrp[0] = 0, .TskInGrp[1] = 0, .TskInGrp[2] = 0, .TskInGrp[3] = 0, .TskInGrp[4] = 0, .TskInGrp[5] = 0, .TskInGrp[6] = 0, .TskInGrp[7] = 0};

static Task_Create_RegList_s TskCrt_RegList = {.num = 0, .list = {.prv = NULL, .nxt = NULL, .data = NULL}};

static volatile Scheduler_State_List scheduler_state = Scheduler_Initial;

/* internal function */
static void Os_ResetTask_Data(Task *task);
static void Os_Set_TaskReady(Task *tsk);
static void Os_Clr_TaskReady(Task *tsk);
static void Os_SchedulerRun(void);
static void Os_TaskExit(void);
static Task *Os_TaskPri_Compare(const Task *tsk_l, const Task *tsk_r);

// first need to know is linux support AT&T formate ASM code
__attribute__((naked)) static void Os_SetPendSVPro(void)
{
    // set pendsv interrupt
    __ASM(".equ NVIC_SYSPRI14, 0xE000ED22");
    __ASM(".equ NVIC_PENDSV_PRI, 0xFF");

    __ASM("LDR      R0, =NVIC_SYSPRI14");
    __ASM("LDR      R1, =NVIC_PENDSV_PRI");
    __ASM("STRB     R1, [R0]");

    // set PSP to 0 to initial context switch call
    __ASM("MOVS     R0, #0");
    __ASM("MSR      PSP, R0");

    // initial MSP to Task_OS_ExpStkBase
    __ASM("LDR      R0, =Task_OS_ExpStkBase");
    __ASM("LDR      R1, [R0]");
    __ASM("MSR      MSP, R1");

    __ASM("BX       LR");
}

__attribute__((naked)) static void Os_TriggerPendSV(void)
{
    __ASM(".equ NVIC_INT_CTRL, 0xE000ED04");
    __ASM(".equ NVIC_PENDSVSET, 0x10000000");

    __ASM("LDR      R0, =NVIC_INT_CTRL");
    __ASM("LDR      R1, =NVIC_PENDSVSET");
    __ASM("STR      R1, [R0]");
    __ASM("BX       LR");
}

void Os_Init(uint32_t TickFRQ)
{
    Kernel_Init();
    Runtime_Config(TickFRQ);

    for (uint8_t g = Task_Group_0; g < Task_Group_Sum; g++)
    {
        for (uint8_t t = Task_Priority_0; t < Task_Priority_Sum; t++)
        {
            /* clear each data in task map ptr */
            Os_ResetTask_Data(&TaskPtr_Map[g][t]);

            /* clear stack depth and stack memory and stack top pointer */
            TaskPtr_Map[g][t]->Stack_Depth = 0;
            TaskPtr_Map[g][t]->TCB.Stack = NULL;
            TaskPtr_Map[g][t]->TCB.Top_Stk_Ptr = NULL;

            TaskPtr_Map[g][t] = NULL;
        }
    }

    TskCrt_RegList.num = 0;
    TskCrt_RegList.list.data = NULL;
    TskCrt_RegList.list.nxt = NULL;
    TskCrt_RegList.list.prv = NULL;

    ReSet_Task_Data(CurRunTsk_Ptr);
    ReSet_Task_Data(NxtRunTsk_Ptr);

    scheduler_state = Scheduler_ready;
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
    task->Exec_status.Exec_cnt = 0;
    task->Exec_status.cpu_opy = 0;
    task->Exec_status.Running_Time = 0;

    task->delay_info.on_delay = false;
    task->delay_info.tsk_hdl = 0;
    task->delay_info.time_unit = 0;

    List_ItemInit(&task->delay_item, &task->delay_info);

    RuntimeObj_Reset(&(task->Exec_status.Exec_Time));

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

/*
 * task caller will not exit
 * if Os run into this func is sort of error happened
 */
static void Os_TaskExit(void)
{
    while (true)
    {
    }
}

// return high priority task pointer
static Task *Os_TaskPri_Compare(const Task *tsk_l, const Task *tsk_r)
{
    if ((tsk_l == NULL) && (tsk_r == NULL))
    {
        return NULL;
    }

    if ((tsk_l == NULL) && (tsk_r != NULL))
    {
        return tsk_r;
    }

    if ((tsk_l != NULL) && (tsk_r == NULL))
    {
        return tsk_l;
    }

    if (GET_TASKGROUP_PRIORITY(tsk_l->priority) < GET_TASKGROUP_PRIORITY(tsk_r->priority))
    {
        return tsk_l;
    }
    else if (GET_TASKGROUP_PRIORITY(tsk_l->priority) > GET_TASKGROUP_PRIORITY(tsk_r->priority))
    {
        return tsk_r;
    }
    else if (GET_TASKGROUP_PRIORITY(tsk_l->priority) == GET_TASKGROUP_PRIORITY(tsk_r->priority))
    {
        if (GET_TASKINGROUP_PRIORITY(tsk_l->priority) < GET_TASKINGROUP_PRIORITY(tsk_r->priority))
        {
            return tsk_l;
        }
        else if (GET_TASKINGROUP_PRIORITY(tsk_l->priority) > GET_TASKINGROUP_PRIORITY(tsk_r->priority))
        {
            return tsk_r;
        }
    }
}

static void Os_TaskExec(Task *tsk_ptr)
{
    SYSTEM_RunTime time_diff;
    RuntimeObj_Reset(&time_diff);

    tsk_ptr = NxtRunTsk_Ptr;

    while (true)
    {
        if (tsk_ptr->Exec_status.State == Task_Ready)
        {
            tsk_ptr->TskFuncUing_US = 0;

            // when task function execute finish reset ready flag of current task in group
            // code down below
            Task_ClearReady(tsk_ptr);

            // set current running task
            CurRunTsk_Ptr = tsk_ptr;

            if (tsk_ptr->Exec_status.Exec_cnt == 0)
            {
                tsk_ptr->Exec_status.Start_Time = Get_CurrentRunningUs();
                tsk_ptr->Exec_status.Exec_Time = tsk_ptr->Exec_status.Start_Time;
            }

            tsk_ptr->Exec_status.State = Task_Running;

            // execute task funtion
            tsk_ptr->Exec_Func(*&tsk_ptr);

            // record task running times
            tsk_ptr->Exec_status.Exec_cnt++;

            // get task total execute time unit in us
            tsk_ptr->Exec_status.Running_Time += tsk_ptr->TskFuncUing_US;
            time_diff = Get_TimeDifference_Between(tsk_ptr->Exec_status.Start_Time, tsk_ptr->Exec_status.Exec_Time);

            tsk_ptr->Exec_status.cpu_opy = tsk_ptr->Exec_status.Running_Time / (float)time_diff;
            tsk_ptr->Exec_status.cpu_opy *= 100;

            tsk_ptr->Exec_status.Exec_Time = Get_TargetRunTime(tsk_ptr->exec_interval_us);

            // get task execute frequence
            if (tsk_ptr->Exec_status.Exec_cnt)
            {
                tsk_ptr->Exec_status.detect_exec_frq = (uint32_t)(tsk_ptr->Exec_status.Exec_cnt / (float)(Get_CurrentRunningS()));
            }

            tsk_ptr->Exec_status.State = Task_Stop;

            // erase currnet runnint task pointer
            CurRunTsk_Ptr = NULL;
        }
    }
}

static void Os_TaskCaller(void)
{
    static uint8_t i = 0;

    // if any task in any group is under ready state
    if (NxtRunTsk_Ptr != NULL)
    {
        i++;
        if (i == TskCrt_RegList.num)
        {
            traverse_start = true;
        }

        // execute task function in function matrix
        Os_TaskExec(NxtRunTsk_Ptr);
    }
}

static int Os_TaskCrtList_TraverseCallback(item_obj *item, void *data, void *arg)
{
    if (data != NULL)
    {
        // get current highest priority task handler AKA NxtRunTsk_Ptr

        if ((scheduler_state == Scheduler_Start) &&
            (((Task *)data)->Exec_status.State == Task_Stop) &&
            (!RuntimeObj_CompareWithCurrent(((Task *)data)->Exec_status.Exec_Time)))
        {
            Task_SetReady((Task *)data);
        }
    }

    return 0;
}
