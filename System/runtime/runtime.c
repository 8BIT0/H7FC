/*
 * Coder : 8_B!T0
 *
 * BitRTOS Core Ticker Code Module
 *
 */
#include "runtime.h"
#include "stm32h743xx.h"
#include <string.h>

/* internal variable */
static RCC_ClocksTypeDef SysFrq;
static runtime_stop_p Runtime_Stop_FuncPtr = NULL;
static Runtime_DataObj_TypeDef RunTime = {
    .tick_callback = NULL,
    .start_callback = NULL,
    .stop_callback = NULL,
};

bool Runtime_SetCallback(Runtime_BaseCallback_TypeList type, runtime_callback_p tick_cb)
{
    switch (type)
    {
    case RtCallback_Type_Start:
        RunTime.start_callback = tick_cb;
        return true;

    case RtCallback_Type_Stop:
        RunTime.stop_callback = tick_cb;
        return true;

    case RtCallback_Type_Tick:
        RunTime.tick_callback = tick_cb;
        return true;

    default:
        return false;
    }
}

Runtime_ModuleState_List Get_RuntimeState(void)
{
    return RunTime.module_state;
}

Tick_Frq Runtime_GetTickFrq(void)
{
    return RunTime.frq;
}

Tick_Base Runtime_GetTickBase(void)
{
    return RunTime.base;
}

bool Runtime_Config(uint32_t tick_frq)
{
    RunTime.module_state = Runtime_Module_Init;
    RunTime.base = RUNTIEM_MAX_TICK_FRQ / tick_frq;
    RunTime.frq = tick_frq;

    SysTick_Config(SystemCoreClock / (RUNTIEM_MAX_TICK_FRQ / RunTime.base)); // 1us system running step
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

    RCC_GetClocksFreq(&SysFrq);

    return true;
}

void Runtime_Start(void)
{
    RunTime.module_state = Runtime_Module_Start;

    if (RunTime.start_callback != NULL)
    {
        RunTime.start_callback();
    }
}

void RuntimeObj_Reset(SYSTEM_RunTime *Obj)
{
    *Obj = 0;
}

SYSTEM_RunTime Get_TimeDifference_Between(SYSTEM_RunTime time_l, SYSTEM_RunTime time_r)
{
    if (time_r >= time_l)
        return (time_r - time_l);

    return (time_l - time_r);
}

bool Runtime_Stop(void)
{
    if (RunTime.tick_state != Runtime_Run_Tick)
    {
        RunTime.module_state = Runtime_Module_Stop;

        if (RunTime.stop_callback != NULL)
        {
            RunTime.stop_callback();
        }

        return true;
    }

    Runtime_Stop_FuncPtr = Runtime_Stop;
    return false;
}

bool Runtime_Tick(void)
{
    if (RunTime.module_state == Runtime_Module_Start)
    {
        RunTime.tick_state = Runtime_Run_Tick;
        RunTime.Use_Us += RunTime.base;
        RunTime.tick_state = Runtime_Run_Wait;

        if (Runtime_Stop_FuncPtr != NULL)
            Runtime_Stop_FuncPtr();

        if (RunTime.tick_callback != NULL)
            RunTime.tick_callback();

        return true;
    }

    return false;
}

inline SYSTEM_RunTime Get_CurrentRunningUs(void)
{
    return RunTime.Use_Us;
}

inline SYSTEM_RunTime Get_CurrentRunningMs(void)
{
    return (RunTime.Use_Us / REAL_MS);
}

SYSTEM_RunTime Get_CurrentRunningS(void)
{
    return (Get_CurrentRunningMs() / REAL_MS);
}

inline SYSTEM_RunTime Get_TimeDifference(uint64_t time_in)
{
    return (RunTime.Use_Us - time_in);
}

inline SYSTEM_RunTime Get_TargetRunTime(uint16_t duration)
{
    return (RunTime.Use_Us + duration);
}

/* return the object addr for the longer one */
/* if EQ_L equal to EQ_R return null pointer */
uint32_t RuntimeObj_Compare(const uint64_t *EQ_L, const uint64_t *EQ_R)
{
    if (*EQ_L > *EQ_R)
    {
        return EQ_L;
    }
    else if (*EQ_L < *EQ_R)
    {
        return EQ_R;
    }

    return NULL;
}

/* input time object compare with current runtime */
/* if input time object >= current runtime return true */
inline bool RuntimeObj_CompareWithCurrent(const uint64_t time_in)
{
    if (time_in >= RunTime.Use_Us)
    {
        return true;
    }

    return false;
}
