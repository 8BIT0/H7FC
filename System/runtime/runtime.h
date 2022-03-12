
#ifndef __RUNTIME_H
#define __RUNTIME_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "system_cfg.h"

#define REAL_1US 1
#define REAL_1MS 1000 * REAL_1US /* 1ms == 1000us */
#define REAL_1S 1000 * REAL_1MS

#define RUNTIME_TICK_FRQ_40K 40000 /* 25us base time */
#define RUNTIME_TICK_FRQ_20K 20000 /* 50us base time */
#define RUNTIME_TICK_FRQ_10K 10000 /* 100us base time */
#define RUNTIME_TICK_FRQ_5K 5000   /* 200us base time */
#define RUNTIME_TICK_FRQ_2K 2000   /* 500us base time */
#define RUNTIME_TICK_FRQ_1K 1000   /* 1ms base time */

typedef enum
{
    RtCallback_Type_Start = 0,
    RtCallback_Type_Stop,
    RtCallback_Type_Tick,
} Runtime_BaseCallback_TypeList;

typedef uint32_t (*runtime_callback_p)(uint64_t Rt);

typedef uint64_t SYSTEM_RunTime;
typedef uint32_t Tick_Frq;
typedef uint32_t Tick_Base;

typedef enum
{
    Runtime_Module_Init,
    Runtime_Module_Start,
    Runtime_Module_Stop,
} Runtime_ModuleState_List;

typedef enum
{
    Runtime_Run_Tick,
    Runtime_Run_Wait,
} Runtime_RunState_List;

/* runtime data block object definition*/
#pragma pack(4)
typedef struct
{
    SYSTEM_RunTime Use_Us;

    Tick_Base base;
    Tick_Frq frq;

    runtime_callback_p start_callback;
    runtime_callback_p stop_callback;
    runtime_callback_p tick_callback;

    Runtime_ModuleState_List module_state;
    Runtime_RunState_List tick_state;
} Runtime_DataObj_TypeDef;
#pragma pack()

bool Runtime_SetCallback(Runtime_BaseCallback_TypeList type, runtime_callback_p stop_cb);
void RuntimeObj_Reset(SYSTEM_RunTime *Obj);

bool Runtime_Config(uint32_t tick_frq);
bool RuntimeObj_CompareWithCurrent(const SYSTEM_RunTime time_in);
bool Runtime_Stop(void);
bool Runtime_Tick(void);
void Runtime_Start(void);

SYSTEM_RunTime Get_CurrentRunningUs(void);
SYSTEM_RunTime Get_CurrentRunningMs(void);
SYSTEM_RunTime Get_CurrentRunningS(void);
SYSTEM_RunTime Get_TimeDifference_ByCurrent(SYSTEM_RunTime time_in);
SYSTEM_RunTime Get_TimeDifference_Between(SYSTEM_RunTime time_l, SYSTEM_RunTime time_r);
SYSTEM_RunTime Get_TargetRunTime(uint16_t duration);
uint32_t RuntimeObj_Compare(const SYSTEM_RunTime *EQ_L, const SYSTEM_RunTime *EQ_R);

Tick_Frq Runtime_GetTickFrq(void);
Tick_Base Runtime_GetTickBase(void);

void Runtime_DelayMs(uint32_t ms);

#endif
