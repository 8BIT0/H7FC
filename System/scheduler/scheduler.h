#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>

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

typedef enum
{
    TaskPriority_Occupy = 1, /* already have created task in current group & priority */
} Task_Error_List;

typedef enum
{
    Task_Ready = 0,
    Task_Running,
    Task_Stop,
    Task_Block,
    Task_Pending,
} Task_State;

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
} Task_Group_Priority;

typedef enum
{
    Scheduler_Initial = 0,
    Scheduler_Prepare,
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

typedef struct
{
    SYSTEM_RunTime Init_Time;
    SYSTEM_RunTime Start_Time;
    SYSTEM_RunTime Exec_Time;

    uint32_t detect_exec_frq;      // detect task running frq
    uint32_t detect_exec_time_arv; // task  average running time
    uint32_t detect_exec_time_max; // task max running time
    uint32_t Exec_Times;
    uint8_t error_code;

    uint32_t totlal_running_time;
    float cpu_opy;
    TASK_STATE State;
} Task_Exec_Status;

typedef struct
{
    Priority_Str priority;
    char *Task_name;

    uint32_t exec_frq;
    uint16_t exec_interval_us;

    Task_Func Exec_Func;
    Task_Exec_Status Exec_status;

    uint32_t Stack_Depth;
    TaskStack_ControlBlock TCB;

    // private variable
    uint32_t TskFuncUing_US; // Cast US time while system Calling the Target task function

    delay_reg delay_info;

    item_obj *item_ptr;
    item_obj delay_item;
} Task;
#pragma pack()

void Os_Init(uint32_t TickFRQ);
void Os_Start(void);

#endif
