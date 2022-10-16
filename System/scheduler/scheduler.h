#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include "runtime.h"
#include "linked_list.h"
#include "kernel.h"

typedef uint32_t Task_Handle;

#define TASK_EXEC_10KHZ 10000
#define TASK_EXEC_8KHZ 8000
#define TASK_EXEC_5KHZ 5000
#define TASK_EXEC_4KHZ 4000
#define TASK_EXEC_2KHZ 2000
#define TASK_EXEC_1KHZ 1000
#define TASK_EXEC_500HZ 500
#define TASK_EXEC_250HZ 250
#define TASK_EXEC_200HZ 200
#define TASK_EXEC_150HZ 150
#define TASK_EXEC_100HZ 100
#define TASK_EXEC_50HZ 50
#define TASK_EXEC_25HZ 25
#define TASK_EXEC_20HZ 20
#define TASK_EXEC_10HZ 10
#define TASK_EXEC_5HZ 5
#define TASK_EXEC_2Hz 2
#define TASK_EXEC_1HZ 1

#define NOERROR 0
#define ERROR_NULLTASK 1

#define TASK_IDLE_GROUP 8
#define TASK_IDLE_PRIORITY 8

typedef uint32_t Task_Handle;
typedef void (*Task_Func)(Task_Handle hdl);
typedef void (*Idle_Callback_Func)(uint8_t *ptr, uint16_t size);
typedef uint32_t *Task_STK_Ptr;

#define TaskHandlerToObj(x) ((Task *)x)
#define DataToDelayRegPtr(x) ((delay_reg *)(x))

typedef enum
{
    Task_Init = 0,
    Task_Ready,
    Task_Running,
    Task_Stop,
    Task_SignalBlock,
    Task_DelayBlock,
    Task_Pending,
} TASK_STATE;

typedef enum
{
    Task_Priority_0 = 0,
    Task_Priority_1,
    Task_Priority_2,
    Task_Priority_3,
    Task_Priority_4,
    Task_Priority_5,
    Task_Priority_6,
    Task_Priority_7,
    Task_Priority_Sum,
} Task_Priority;

typedef enum
{
    Task_Group_0 = 0,
    Task_Group_1,
    Task_Group_2,
    Task_Group_3,
    Task_Group_4,
    Task_Group_5,
    Task_Group_6,
    Task_Group_7,
    Task_Group_Sum,
} Task_Group;

typedef enum
{
    Scheduler_Initial = 0,
    Scheduler_ready,
    Scheduler_Start,
} Scheduler_State_List;

#pragma pack(1)
typedef union
{
    uint8_t Flg;

    struct
    {
        uint8_t bit_0 : 1;
        uint8_t bit_1 : 1;
        uint8_t bit_2 : 1;
        uint8_t bit_3 : 1;
        uint8_t bit_4 : 1;
        uint8_t bit_5 : 1;
        uint8_t bit_6 : 1;
        uint8_t bit_7 : 1;
    } Bit;
} Flg_template;

typedef struct
{
    Flg_template Grp;
    Flg_template TskInGrp[Task_Group_Sum];
} TaskMap_TypeDef;
#pragma pack()

#pragma pack(4)
typedef struct
{
    Task_STK_Ptr Top_Stk_Ptr;
    Task_STK_Ptr Stack;
} TaskStack_ControlBlock;

typedef struct
{
    SYSTEM_RunTime Exec_Time;
    SYSTEM_RunTime Start_Time;

    uint32_t detect_exec_frq; // detect task running frq
    uint32_t Exec_cnt;
    uint8_t error_code;

    SYSTEM_RunTime Running_Time;
    float cpu_opy;
} Task_Exec_Status;

typedef struct
{
    uint8_t num;
    list_obj list;
} Task_List_s;

typedef Task_List_s Idle_Callback_List_s;

typedef struct
{
    uint8_t *ptr;
    uint16_t size;
} Os_Idle_DataStream_TypeDef;

typedef struct
{
    Os_Idle_DataStream_TypeDef stream;
    Idle_Callback_Func callback;

    uint64_t exe_cnt;
    list_obj idle_item;
} Os_IdleObj_TypeDef;

typedef struct
{
    TASK_STATE State;

    uint8_t priority;
    char *Task_name;

    uint32_t exec_frq;
    uint16_t exec_interval_us;

    Task_Func Exec_Func;
    Task_Exec_Status Exec_status;

    uint32_t Stack_Depth;
    TaskStack_ControlBlock TCB;

    // private variable
    uint32_t TskFuncUing_US; // Cast US time while system Calling the Target task function

    item_obj *item_ptr;
} Task;
#pragma pack()

void Os_Start(void);
void Os_LoadFirstTask(void);
void Os_Init(uint32_t TickFRQ);
Scheduler_State_List Os_State(void);
Task_Handle Os_CreateTask(const char *name, uint32_t frq, Task_Group group, Task_Priority priority, Task_Func func, uint32_t StackDepth);
void Os_SwitchContext(void);
void Os_TaskDelay_Ms(Task_Handle hdl, uint32_t Ms);

bool Os_Regist_IdleObj(Os_IdleObj_TypeDef *obj, Os_Idle_DataStream_TypeDef stream, Idle_Callback_Func idle_cb);
bool Os_Unregist_IdleObj(Os_IdleObj_TypeDef *obj);

#endif
