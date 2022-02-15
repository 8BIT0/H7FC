#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>

typedef uint32_t Task_Handle;

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
#pragma pack()

#endif
