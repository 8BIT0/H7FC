#include "scheduler.h"
#include "kernel.h"
#include "runtime.h"
#include "linked_list.h"
#include "mmu.h"

// coder: 8_B!T0
// bref:
// estabishment a task running system with priority calling functional
// alway calling the highest priority function in all function under the ready table

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
#define IS_IDLETASK(x) x & 0x80
#define IDLE_TASK_STACK 2048
#define IDLE_TASK_PRIORITY 0x80

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

static volatile Task *CurRunTsk_Ptr = NULL;
static volatile Task *NxtRunTsk_Ptr = NULL;
static volatile TaskStack_ControlBlock CurTsk_TCB;
static volatile TaskStack_ControlBlock NxtTsk_TCB;

static volatile TaskMap_TypeDef TskHdl_RdyMap = {.Grp = 0, .TskInGrp[0] = 0, .TskInGrp[1] = 0, .TskInGrp[2] = 0, .TskInGrp[3] = 0, .TskInGrp[4] = 0, .TskInGrp[5] = 0, .TskInGrp[6] = 0, .TskInGrp[7] = 0};
static volatile TaskMap_TypeDef TskHdl_PndMap = {.Grp = 0, .TskInGrp[0] = 0, .TskInGrp[1] = 0, .TskInGrp[2] = 0, .TskInGrp[3] = 0, .TskInGrp[4] = 0, .TskInGrp[5] = 0, .TskInGrp[6] = 0, .TskInGrp[7] = 0};
static volatile TaskMap_TypeDef TskHdl_BlkMap = {.Grp = 0, .TskInGrp[0] = 0, .TskInGrp[1] = 0, .TskInGrp[2] = 0, .TskInGrp[3] = 0, .TskInGrp[4] = 0, .TskInGrp[5] = 0, .TskInGrp[6] = 0, .TskInGrp[7] = 0};

static volatile Task_List_s TskCrt_RegList = {.num = 0, .list = {.prv = NULL, .nxt = NULL, .data = NULL}};
static volatile Task_List_s TskRdy_RegList = {.num = 0, .list = {.prv = NULL, .nxt = NULL, .data = NULL}};

static volatile Scheduler_State_List scheduler_state = Scheduler_Initial;

static Task *TaskPtr_Map[Task_Group_Sum][Task_Priority_Sum] = {{NULL}};
static Task *Idle_Task;
static Idle_Callback_List_s Idle_List;

/* internal function */
static void Os_ResetTask_Data(Task *task);
static void Os_Set_TaskReady(Task *tsk);
static void Os_Clr_TaskReady(Task *tsk);
static void Os_SchedulerRun(SYSTEM_RunTime Rt);
static void Os_TaskExit(void);
static Task *Os_TaskPri_Compare(const Task *tsk_l, const Task *tsk_r);
static int Os_TaskCrtList_TraverseCallback(item_obj *item, void *data, void *arg);
static void Os_TaskCaller(void);
static Task *Os_Get_HighestRank_PndTask(void);
static Task *Os_Get_HighestRank_RdyTask(void);
static bool Os_CreateIdle(void);
static void Os_Idle(void);
void Os_SwitchTaskStack(void);

__attribute__((naked)) void Os_LoadFirstTask(void)
{
    __ASM("LDR	  R3, =CurTsk_TCB");
    __ASM("LDR    R1, [R3]");
    __ASM("LDR    R0, [R1]");

    __ASM("LDMIA  R0!, {R4-R11, R14}");

    /******************************  FPU SECTION  *********************************/
    __ASM("TST      R14, #0x10");
    __ASM("IT       EQ");
    __ASM("VLDMIAEQ R0!, {s16-s31}");
    /******************************  FPU SECTION  *********************************/

    __ASM("MSR    PSP, R0");
    __ASM("ISB");
    __ASM("BX     R14");
    __ASM(".ALIGN 4");
}

__attribute__((naked)) void Os_SwitchContext(void)
{
    __ASM("MRS      R0, PSP");
    __ASM("ISB");

    __ASM("LDR      R3, CurrentTCBConst_Tmp");
    __ASM("LDR      R2, [R3]");

    /******************************  FPU SECTION  *********************************/
    __ASM("TST      R14, #0x10");
    __ASM("IT       EQ");
    __ASM("VSTMDBEQ R0!, {s16-s31}");
    /******************************  FPU SECTION  *********************************/

    __ASM("STMDB    R0!, {R4-R11, R14}");
    __ASM("STR      R0, [R2]");

    __ASM("STMDB    SP!, {R0, R3}");
    __ASM("MOV      R0, %0" ::"i"(SYSCALL_INTERRUPT_PRIORITY));
    __ASM("MSR      BASEPRI, R0");

    __ASM("DSB");
    __ASM("ISB");

    __ASM("BL       Os_SwitchTaskStack");

    __ASM("MOV      R0, #0");
    __ASM("MSR      BASEPRI, R0");
    __ASM("LDMIA    SP!, {R0, R3}");

    __ASM("LDR      R1, [R3]");
    __ASM("LDR      R0, [R1]");

    __ASM("LDMIA    R0!, {R4-R11, R14}");

    /******************************  FPU SECTION  *********************************/
    __ASM("TST      R14, #0x10");
    __ASM("IT       EQ");
    __ASM("VLDMIAEQ R0!, {s16-s31}");
    /******************************  FPU SECTION  *********************************/

    __ASM("MSR      PSP,R0");
    __ASM("ISB");
    __ASM("BX       R14");

    __ASM("CurrentTCBConst_Tmp: .word CurTsk_TCB");
    __ASM(".ALIGN 4");
}

void Os_SwitchTaskStack(void)
{
    CurTsk_TCB = NxtTsk_TCB;
    CurRunTsk_Ptr = NxtRunTsk_Ptr;
}

static void Os_Set_TaskStk(Task *tsk)
{
    uint32_t *Tsk_Ptr_tmp = NULL;

    memset(tsk->TCB.Stack, NULL, tsk->Stack_Depth * sizeof(uint32_t));

    Tsk_Ptr_tmp = &tsk->TCB.Stack + (tsk->Stack_Depth - (uint32_t)1);
    Tsk_Ptr_tmp = (uint32_t *)((uint32_t)(Tsk_Ptr_tmp)&0XFFFFFFF8ul);

    Tsk_Ptr_tmp--;
    *Tsk_Ptr_tmp = 0x01000000uL; /* xPSR */

    Tsk_Ptr_tmp--;
    *Tsk_Ptr_tmp = ((uint32_t)Os_TaskCaller) & 0xfffffffeUL; /* PC */

    Tsk_Ptr_tmp--;
    *Tsk_Ptr_tmp = (uint32_t)Os_TaskExit; /* LR */

    /* Save code space by skipping register initialisation. */
    Tsk_Ptr_tmp -= 5;              /* R12, R3, R2 and R1. */
    *Tsk_Ptr_tmp = (uint32_t)NULL; /* R0 */

    /* A save method is being used that requires each task to maintain its
        own exec return value. */
    Tsk_Ptr_tmp--;
    *Tsk_Ptr_tmp = 0xfffffffd;

    Tsk_Ptr_tmp -= 8; /* R11, R10, R9, R8, R7, R6, R5 and R4. */

    // set task stack top pointer
    tsk->TCB.Top_Stk_Ptr = Tsk_Ptr_tmp; //&Tsk_Ptr_tmp
}

void Os_Init(uint32_t TickFRQ)
{
    while (!Kernel_Init())
        ;

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

    Os_ResetTask_Data(CurRunTsk_Ptr);
    Os_ResetTask_Data(NxtRunTsk_Ptr);

    Runtime_Config(TickFRQ);

    scheduler_state = Scheduler_ready;
}

Scheduler_State_List Os_State(void)
{
    return scheduler_state;
}

void Os_Start(void)
{
    NxtRunTsk_Ptr = Os_Get_HighestRank_RdyTask();

    if (NxtRunTsk_Ptr != NULL)
    {
        NxtTsk_TCB.Top_Stk_Ptr = &NxtRunTsk_Ptr->TCB.Top_Stk_Ptr;
        NxtTsk_TCB.Stack = NxtRunTsk_Ptr->TCB.Stack;

        CurTsk_TCB = NxtTsk_TCB;
    }

    scheduler_state = Scheduler_Start;

    /* before start we need create a idle task */
    Os_CreateIdle();

    // DrvTimer.ctl(DrvTimer_Counter_SetState, (uint32_t)&SysTimerObj, ENABLE);
    Kernel_EnablePendSV();
    Runtime_SetCallback(RtCallback_Type_Tick, Os_SchedulerRun);

    // trigger SVC Handler
    Kernel_LoadProcess();
}

static void Os_Idle_List_TraverseCallback(item_obj *item, void *ptr_1, void *ptr_2)
{
    Os_IdleObj_TypeDef *idle_obj = NULL;
    if ((item == NULL) || (ptr_1 == NULL))
        return;

    idle_obj = ((Os_IdleObj_TypeDef *)ptr_1);

    idle_obj->callback(idle_obj->stream.ptr, idle_obj->stream.size);
    idle_obj->exe_cnt++;
}

static void Os_Idle(void)
{
    volatile SYSTEM_RunTime Rt = 0;
    static volatile uint64_t idle_cnt = 0;

    Idle_Task->State = Task_Ready;

    if (Idle_List.num != 0)
    {
        if (Rt != Get_CurrentRunningUs())
        {
            Rt = Get_CurrentRunningUs();
            idle_cnt++;
        }
    }
    else
    {
        List_traverse(&Idle_List.list, Os_Idle_List_TraverseCallback, NULL, pre_callback);
    }
}

bool Os_Regist_IdleObj(Os_IdleObj_TypeDef *obj, Os_Idle_DataStream_TypeDef stream, Idle_Callback_Func idle_cb)
{
    if (obj == NULL)
        return false;

    obj->stream = stream;
    obj->callback = idle_cb;

    List_ItemInit(&obj->idle_item, obj);

    if (Idle_List.num == 0)
        List_Init(&Idle_List.list, &(obj->idle_item), by_order, NULL);

    List_Insert_Item(&Idle_List.list, &(obj->idle_item));

    return true;
}

bool Os_Unregist_IdleObj(Os_IdleObj_TypeDef *obj)
{
    if (Idle_List.num)
    {
        List_Delete_Item(&(obj->idle_item), NULL);
        Idle_List.num--;
    }
}

static bool Os_CreateIdle(void)
{
    /* for idle task we don`t need priority or other state */
    Idle_Task = (Task *)MMU_Malloc(sizeof(Task));

    if (Idle_Task == NULL)
        return false;

    Idle_Task->Task_name = "Idle";
    Idle_Task->priority = IDLE_TASK_PRIORITY;
    Idle_Task->Exec_Func = Os_Idle;

    Idle_Task->Stack_Depth = IDLE_TASK_STACK;
    Idle_Task->TCB.Stack = (uint32_t *)MMU_Malloc(IDLE_TASK_STACK * sizeof(uint32_t));

    if (Idle_Task->TCB.Stack != NULL)
    {
        Os_Set_TaskStk(Idle_Task);
    }
    else
        return false;

    Idle_List.num = 0;

    return true;
}

Task_Handle Os_CreateTask(const char *name, uint32_t frq, Task_Group group, Task_Priority priority, Task_Func func, uint32_t StackDepth)
{
    Task_Handle handle;
    uint16_t task_name_len = strlen(name);
    uint32_t *Tsk_Ptr_tmp = NULL;

    // already have task in current group and priority in task pointer matrix
    if (TaskPtr_Map[group][priority] != NULL)
        return NULL;

    // request a memory space for Task_Ptr contain
    TaskPtr_Map[group][priority] = (Task *)MMU_Malloc(sizeof(Task));

    if (TaskPtr_Map[group][priority] == NULL)
        return NULL;

    // record Task_Ptr poiner`s address
    handle = *&TaskPtr_Map[group][priority];

    TaskPtr_Map[group][priority]->Task_name = name;

    TaskPtr_Map[group][priority]->exec_frq = frq;
    TaskPtr_Map[group][priority]->exec_interval_us = SCHEDULER_TIMEBASE / frq;
    TaskPtr_Map[group][priority]->Exec_Func = func;

    TaskPtr_Map[group][priority]->priority = (group << 3) | priority;

    // request memory space for task stack
    TaskPtr_Map[group][priority]->Stack_Depth = StackDepth;
    TaskPtr_Map[group][priority]->TCB.Stack = (uint32_t *)MMU_Malloc(StackDepth * sizeof(uint32_t));

    if (TaskPtr_Map[group][priority]->TCB.Stack != NULL)
    {
        Os_Set_TaskStk(TaskPtr_Map[group][priority]);
    }
    else
        return NULL;

    // reset single loop running us
    TaskPtr_Map[group][priority]->TskFuncUing_US = 0;

    // reset task cpu occupy data
    TaskPtr_Map[group][priority]->Exec_status.cpu_opy = 0;
    TaskPtr_Map[group][priority]->Exec_status.Running_Time = 0;

    // set current group flag to ready
    TskHdl_RdyMap.Grp.Flg |= 1 << GET_TASKGROUP_PRIORITY(TaskPtr_Map[group][priority]->priority);
    // set current task under this group flag to ready
    TskHdl_RdyMap.TskInGrp[GET_TASKGROUP_PRIORITY(TaskPtr_Map[group][priority]->priority)].Flg |= 1 << GET_TASKINGROUP_PRIORITY(TaskPtr_Map[group][priority]->priority);

    TaskPtr_Map[group][priority]->Exec_status.detect_exec_frq = 0;

    RuntimeObj_Reset(&(TaskPtr_Map[group][priority]->Exec_status.Exec_Time));
    RuntimeObj_Reset(&(TaskPtr_Map[group][priority]->Exec_status.Start_Time));

    TaskPtr_Map[group][priority]->Exec_status.Exec_cnt = 0;
    TaskPtr_Map[group][priority]->Exec_status.error_code = NOERROR;

    Os_Set_TaskReady(TaskPtr_Map[group][priority]);

    TaskPtr_Map[group][priority]->item_ptr = (item_obj *)MMU_Malloc(sizeof(item_obj));
    if (TaskPtr_Map[group][priority]->item_ptr == NULL)
        return NULL;

    List_ItemInit(TaskPtr_Map[group][priority]->item_ptr, TaskPtr_Map[group][priority]);
    if (TskCrt_RegList.num == 0)
    {
        List_Init(&TskCrt_RegList.list, TaskPtr_Map[group][priority]->item_ptr, by_condition, Os_TaskPri_Compare);
    }
    else
    {
        List_Insert_Item(&TskCrt_RegList.list, TaskPtr_Map[group][priority]->item_ptr);
    }

    TskCrt_RegList.num++;

    return handle;
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

    RuntimeObj_Reset(&(task->Exec_status.Exec_Time));

    task->State = Task_Init;
}

static void Os_Set_TaskReady(Task *tsk)
{
    uint8_t grp_id = GET_TASKGROUP_PRIORITY(tsk->priority);
    uint8_t tsk_id = GET_TASKINGROUP_PRIORITY(tsk->priority);

    // set current group flag to ready
    TskHdl_RdyMap.Grp.Flg |= 1 << grp_id;
    // set current task under this group flag to ready
    TskHdl_RdyMap.TskInGrp[grp_id].Flg |= 1 << tsk_id;

    tsk->State = Task_Ready;
}

void Os_Set_TaskPending(Task *tsk)
{
    uint8_t grp_id = GET_TASKGROUP_PRIORITY(tsk->priority);
    uint8_t tsk_id = GET_TASKINGROUP_PRIORITY(tsk->priority);

    // set current group pending
    TskHdl_PndMap.Grp.Flg |= 1 << grp_id;
    // set current task under this group flag to ready
    TskHdl_PndMap.TskInGrp[grp_id].Flg |= 1 << tsk_id;

    // set task state
    tsk->State = Task_Pending;
}

static void Os_Set_TaskBlock(Task *tsk, TASK_STATE state)
{
    uint8_t grp_id = GET_TASKGROUP_PRIORITY(tsk->priority);
    uint8_t tsk_id = GET_TASKINGROUP_PRIORITY(tsk->priority);

    // set current group pending
    TskHdl_BlkMap.Grp.Flg |= 1 << grp_id;
    // set current task under this group flag to ready
    TskHdl_BlkMap.TskInGrp[grp_id].Flg |= 1 << tsk_id;

    // set task state
    tsk->State = state;
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

static void Os_Clr_TaskPending(Task *tsk)
{
    uint8_t grp_id = GET_TASKGROUP_PRIORITY(tsk->priority);
    uint8_t tsk_id = GET_TASKINGROUP_PRIORITY(tsk->priority);

    TskHdl_PndMap.TskInGrp[grp_id].Flg &= ~(1 << tsk_id);
    if (TskHdl_PndMap.TskInGrp[grp_id].Flg == 0)
    {
        TskHdl_PndMap.Grp.Flg &= ~(1 << grp_id);
    }
}

static void Os_Clr_TaskBlock(Task *tsk)
{
    uint8_t grp_id = GET_TASKGROUP_PRIORITY(tsk->priority);
    uint8_t tsk_id = GET_TASKINGROUP_PRIORITY(tsk->priority);

    TskHdl_BlkMap.TskInGrp[grp_id].Flg &= ~(1 << tsk_id);
    if (TskHdl_BlkMap.TskInGrp[grp_id].Flg == 0)
    {
        TskHdl_BlkMap.Grp.Flg &= ~(1 << grp_id);
    }
}

static void Os_SchedulerRun(SYSTEM_RunTime Rt)
{
    SYSTEM_RunTime CurRt_US = Rt;
    Task *TskPtr_Tmp = NULL;

    if (TskCrt_RegList.num)
    {
        /* check task state ready or not */
        List_traverse(&TskCrt_RegList.list, Os_TaskCrtList_TraverseCallback, &CurRt_US, pre_callback);
    }

    TskPtr_Tmp = Os_TaskPri_Compare(Os_TaskPri_Compare(Os_Get_HighestRank_RdyTask(), Os_Get_HighestRank_PndTask()), CurRunTsk_Ptr);

    if (TskPtr_Tmp != CurRunTsk_Ptr)
    {
        if (TskPtr_Tmp != NULL)
        {
            if (TskPtr_Tmp->State == Task_Pending)
            {
                /* got problem after priority preemption */

                Os_Clr_TaskPending(TskPtr_Tmp);
                Os_Set_TaskReady(TskPtr_Tmp);
            }

            if ((CurRunTsk_Ptr != NULL) && (CurRunTsk_Ptr != Idle_Task) && (CurRunTsk_Ptr->State == Task_Running))
            {
                /* set current task in pending list */
                Os_Set_TaskPending(CurRunTsk_Ptr);
            }

            if ((TskPtr_Tmp->State != Task_DelayBlock) && (TskPtr_Tmp->State != Task_SignalBlock))
            {
                NxtRunTsk_Ptr = TskPtr_Tmp;

                NxtTsk_TCB.Top_Stk_Ptr = &NxtRunTsk_Ptr->TCB.Top_Stk_Ptr;
                NxtTsk_TCB.Stack = NxtRunTsk_Ptr->TCB.Stack;

                /* trigger pendsv to switch task */
                Kernel_TriggerPendSV();
            }
        }
    }
    else
    {
        if (((TskPtr_Tmp == NULL) &&
             (CurRunTsk_Ptr == NULL)) ||
            (TskPtr_Tmp->State == Task_DelayBlock) ||
            (TskPtr_Tmp->State == Task_SignalBlock))
        {
            Idle_Task->State = Task_Ready;

            NxtRunTsk_Ptr = Idle_Task;

            NxtTsk_TCB.Top_Stk_Ptr = &Idle_Task->TCB.Top_Stk_Ptr;
            NxtTsk_TCB.Stack = Idle_Task->TCB.Stack;

            /* trigger pendsv to switch task */
            Kernel_TriggerPendSV();
        }
    }
}

/* still got bug down below */
void Os_TaskDelay_Ms(Task_Handle hdl, uint32_t Ms)
{
    Task *NxtTsk_Tmp = NULL;

    SYSTEM_RunTime delay_tick_base = (Ms * REAL_1MS);

    /* set task next ready time */
    TaskHandlerToObj(hdl)->Exec_status.Exec_Time += delay_tick_base;

    /* clear from ready list */
    Os_Clr_TaskReady(TaskHandlerToObj(hdl));
    Os_Set_TaskBlock(TaskHandlerToObj(hdl), Task_DelayBlock);

    NxtTsk_Tmp = Os_Get_HighestRank_RdyTask();

    if (NxtTsk_Tmp == NULL)
    {
        NxtRunTsk_Ptr = Idle_Task;
        Idle_Task->State = Task_Ready;

        NxtTsk_TCB.Top_Stk_Ptr = &Idle_Task->TCB.Top_Stk_Ptr;
        NxtTsk_TCB.Stack = Idle_Task->TCB.Stack;
    }
    else
    {
        NxtRunTsk_Ptr = NxtTsk_Tmp;

        NxtTsk_TCB.Top_Stk_Ptr = &NxtTsk_Tmp->TCB.Top_Stk_Ptr;
        NxtTsk_TCB.Stack = NxtTsk_Tmp->TCB.Stack;
    }

    /* trigger pendsv to switch task */
    /* do not anticipate scheduler been triggerd when it happend  */
    Kernel_TriggerPendSV();
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

static Task *Os_Get_HighestRank_RdyTask(void)
{
    uint8_t grp_id = 0;
    uint8_t tsk_id = 0;

    if (TskHdl_RdyMap.Grp.Flg)
    {
        // find group
        grp_id = Task_Priority_List[TskHdl_RdyMap.Grp.Flg];
        // find task in group
        tsk_id = Task_Priority_List[TskHdl_RdyMap.TskInGrp[grp_id].Flg];
    }
    else
        return NULL;

    if (TaskPtr_Map[grp_id][tsk_id] != NULL)
    {
        return TaskPtr_Map[grp_id][tsk_id];
    }
    else
    {
        TskHdl_RdyMap.Grp.Flg &= ~(1 << grp_id);
        TskHdl_RdyMap.TskInGrp[grp_id].Flg &= ~(1 << tsk_id);
        return NULL;
    }
}

static Task *Os_Get_HighestRank_PndTask(void)
{
    uint8_t grp_id;
    uint8_t tsk_id;

    if (TskHdl_PndMap.Grp.Flg)
    {
        // find group
        grp_id = Task_Priority_List[TskHdl_PndMap.Grp.Flg];
        // find task in group
        tsk_id = Task_Priority_List[TskHdl_PndMap.TskInGrp[grp_id].Flg];
    }
    else
        return NULL;

    if (TaskPtr_Map[grp_id][tsk_id] != NULL)
    {
        return TaskPtr_Map[grp_id][tsk_id];
    }
    else
    {
        TskHdl_PndMap.Grp.Flg &= ~(1 << grp_id);
        TskHdl_PndMap.TskInGrp[grp_id].Flg &= ~(1 << tsk_id);
        return NULL;
    }
}

// return high priority task pointer
static Task *Os_TaskPri_Compare(const Task *tsk_l, const Task *tsk_r)
{
    volatile uint8_t grp_l = GET_TASKGROUP_PRIORITY(tsk_l->priority);
    volatile uint8_t grp_r = GET_TASKGROUP_PRIORITY(tsk_r->priority);
    volatile uint8_t pri_l = GET_TASKINGROUP_PRIORITY(tsk_l->priority);
    volatile uint8_t pri_r = GET_TASKINGROUP_PRIORITY(tsk_r->priority);

    if ((tsk_l == NULL) && (tsk_r == NULL))
    {
        return NULL;
    }

    if ((tsk_l == NULL) && (tsk_r != NULL))
    {
        if ((tsk_r->State != Task_DelayBlock) && (tsk_r->State != Task_SignalBlock))
        {
            return tsk_r;
        }
        else
            return NULL;
    }

    if ((tsk_l != NULL) && (tsk_r == NULL))
    {
        if ((tsk_l->State != Task_DelayBlock) && (tsk_l->State != Task_SignalBlock))
        {
            return tsk_l;
        }
        else
            return NULL;
    }

    if (grp_l < grp_r)
    {
        if ((tsk_l->State != Task_DelayBlock) && (tsk_l->State != Task_SignalBlock))
        {
            return tsk_l;
        }

        if ((tsk_r->State != Task_DelayBlock) && (tsk_r->State != Task_SignalBlock))
        {
            return tsk_r;
        }

        return NULL;
    }
    else if (grp_l > grp_r)
    {
        if ((tsk_r->State != Task_DelayBlock) && (tsk_r->State != Task_SignalBlock))
        {
            return tsk_r;
        }

        if ((tsk_l->State != Task_DelayBlock) && (tsk_l->State != Task_SignalBlock))
        {
            return tsk_l;
        }

        return NULL;
    }
    else if (grp_l == grp_r)
    {
        if (pri_l < pri_r)
        {
            if ((tsk_l->State != Task_DelayBlock) && (tsk_l->State != Task_SignalBlock))
            {
                return tsk_l;
            }

            if ((tsk_r->State != Task_DelayBlock) && (tsk_r->State != Task_SignalBlock))
            {
                return tsk_r;
            }

            return NULL;
        }
        else if (pri_l > pri_r)
        {
            if ((tsk_r->State != Task_DelayBlock) && (tsk_r->State != Task_SignalBlock))
            {
                return tsk_r;
            }

            if ((tsk_l->State != Task_DelayBlock) && (tsk_l->State != Task_SignalBlock))
            {
                return tsk_l;
            }

            return NULL;
        }
        else
        {
            if ((tsk_r->State != Task_DelayBlock) && (tsk_r->State != Task_SignalBlock))
            {
                return tsk_r;
            }

            if ((tsk_l->State != Task_DelayBlock) && (tsk_l->State != Task_SignalBlock))
            {
                return tsk_l;
            }

            return NULL;
        }
    }
}

static void Os_TaskExec(Task *tsk_ptr)
{
    volatile SYSTEM_RunTime time_diff;
    volatile SYSTEM_RunTime func_cast;
    RuntimeObj_Reset(&time_diff);

    if (tsk_ptr->State == Task_Ready)
    {
        tsk_ptr->TskFuncUing_US = 0;

        // when task function execute finish reset ready flag of current task in group
        // code down below
        Os_Clr_TaskReady(tsk_ptr);

        // set current running task
        CurRunTsk_Ptr = tsk_ptr;

        if ((tsk_ptr->Exec_status.Exec_cnt == 0) && (tsk_ptr != Idle_Task))
        {
            tsk_ptr->Exec_status.Start_Time = Get_CurrentRunningUs();
            tsk_ptr->Exec_status.Exec_Time = tsk_ptr->Exec_status.Start_Time;
        }

        if (tsk_ptr->Exec_Func != NULL)
        {
            tsk_ptr->State = Task_Running;

            func_cast = Get_CurrentRunningUs();
            tsk_ptr->Exec_status.Exec_Time = Get_CurrentRunningUs();
            // execute task funtion
            tsk_ptr->Exec_Func(*&tsk_ptr);
            func_cast = Get_CurrentRunningUs() - func_cast;
        }
        else
        {
            tsk_ptr->State = Task_Stop;
            return;
        }

        if (tsk_ptr == Idle_Task)
            return;

        // record task running times
        tsk_ptr->Exec_status.Exec_cnt++;

        // get task total execute time unit in us
        tsk_ptr->Exec_status.Running_Time += tsk_ptr->TskFuncUing_US;
        time_diff = Get_TimeDifference_Between(tsk_ptr->Exec_status.Start_Time, tsk_ptr->Exec_status.Exec_Time);
        // tsk_ptr->Exec_status.Exec_Time = Get_TargetRunTime(tsk_ptr->exec_interval_us);
        tsk_ptr->Exec_status.Exec_Time += tsk_ptr->exec_interval_us;

        tsk_ptr->Exec_status.cpu_opy = tsk_ptr->Exec_status.Running_Time / (float)time_diff;
        tsk_ptr->Exec_status.cpu_opy *= 100;

        // get task execute frequence
        if (tsk_ptr->Exec_status.Exec_cnt)
        {
            tsk_ptr->Exec_status.detect_exec_frq = (uint32_t)(tsk_ptr->Exec_status.Exec_cnt / (float)(Get_CurrentRunningS()));
        }

        __DSB();

        tsk_ptr->State = Task_Stop;
    }
}

static void Os_TaskCaller(void)
{
    // if any task in any group is under ready state
    while (true)
    {
        if (NxtRunTsk_Ptr != NULL)
        {
            // execute task function in function matrix
            Os_TaskExec(NxtRunTsk_Ptr);

            // erase currnet runnint task pointer
            if (CurRunTsk_Ptr != Idle_Task)
            {
                CurRunTsk_Ptr = NULL;
            }

            // get net task
            Os_SchedulerRun(Get_CurrentRunningUs());
        }
    }
}

static int Os_TaskCrtList_TraverseCallback(item_obj *item, void *data, void *arg)
{
    if (data != NULL)
    {
        // get current highest priority task handler AKA NxtRunTsk_Ptr
        if ((scheduler_state == Scheduler_Start) &&
            ((((Task *)data)->State == Task_Ready) ||
             (((Task *)data)->State == Task_Stop) ||
             (((Task *)data)->State == Task_DelayBlock)) &&
            (RuntimeObj_CompareWithCurrent(((Task *)data)->Exec_status.Exec_Time)))
        {
            Os_Set_TaskReady((Task *)data);
        }
    }

    return 0;
}
