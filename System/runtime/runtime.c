/*
 * Coder : 8_B!T0
 *
 * BitRTOS Core Ticker Code Module
 *
 */
#include "runtime.h"
#include "system_cfg.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc.h"

/* internal variable */
static volatile uint32_t sysclock = 0;
static uint32_t Runtime_DelayUs = 0;
static volatile Runtime_DataObj_TypeDef RunTime = {
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

/**
 \param [in] : tick frequence
                RUNTIME_TICK_FRQ_20K
                RUNTIME_TICK_FRQ_10K
                RUNTIME_TICK_FRQ_5K
                RUNTIME_TICK_FRQ_2K
                RUNTIME_TICK_FRQ_1K
**/
bool Runtime_Config(uint32_t tick_frq)
{
    uint32_t frq = RUNTIME_TICK_FRQ_40K;

    if (tick_frq == RUNTIME_TICK_FRQ_40K)
    {
        frq = RUNTIME_TICK_FRQ_40K;
    }
    else if (tick_frq == RUNTIME_TICK_FRQ_10K)
    {
        frq = RUNTIME_TICK_FRQ_10K;
    }
    else if (tick_frq == RUNTIME_TICK_FRQ_5K)
    {
        frq = RUNTIME_TICK_FRQ_5K;
    }
    else if (tick_frq == RUNTIME_TICK_FRQ_2K)
    {
        frq = RUNTIME_TICK_FRQ_2K;
    }
    else if (tick_frq == RUNTIME_TICK_FRQ_1K)
    {
        frq = RUNTIME_TICK_FRQ_1K;
    }

    RunTime.module_state = Runtime_Module_Init;
    RunTime.base = REAL_1S / frq;
    RunTime.frq = tick_frq;

    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / RunTime.frq);

    sysclock = HAL_RCC_GetSysClockFreq();

    return true;
}

void Runtime_Start(void)
{
    RunTime.module_state = Runtime_Module_Start;

    RunTime.Use_Us = 0;

    if (RunTime.start_callback != NULL)
    {
        RunTime.start_callback(0);
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
            RunTime.stop_callback(0);

        return true;
    }

    return false;
}

bool Runtime_Tick(void)
{
    if (RunTime.module_state == Runtime_Module_Start)
    {
        RunTime.tick_state = Runtime_Run_Tick;
        RunTime.Use_Us += RunTime.base;
        RunTime.tick_state = Runtime_Run_Wait;

        if (RunTime.tick_callback != NULL)
            RunTime.tick_callback(RunTime.Use_Us);

        if (Runtime_DelayUs)
            Runtime_DelayUs--;

        return true;
    }

    return false;
}

SYSTEM_RunTime Get_CurrentRunningUs(void)
{
    return RunTime.Use_Us;
}

SYSTEM_RunTime Get_CurrentRunningMs(void)
{
    return (RunTime.Use_Us / REAL_1MS);
}

SYSTEM_RunTime Get_CurrentRunningS(void)
{
    return (Get_CurrentRunningMs() / REAL_1MS);
}

SYSTEM_RunTime Get_TimeDifference(uint64_t time_in)
{
    return (RunTime.Use_Us - time_in);
}

SYSTEM_RunTime Get_TargetRunTime(uint16_t duration)
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
/* if input time object equl to current runtime return true */
bool RuntimeObj_CompareWithCurrent(const uint64_t time_in)
{
    if (time_in == RunTime.Use_Us)
    {
        return true;
    }

    return false;
}

void Runtime_DelayMs(uint32_t ms)
{
    Runtime_DelayUs = ms * (REAL_1MS / RunTime.base);

    while (Runtime_DelayUs)
        ;
}
