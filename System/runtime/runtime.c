/*
 * Coder : 8_B!T0
 *
 * BitRTOS Core Ticker Code Module
 *
 */
#include "runtime.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc.h"

static void SystemClock_Config(void);
static void Error_Handler(void);

/* internal variable */
static uint32_t sysclock = 0;
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
                RUNTIME_TICK_FRQ_20K (default)
                RUNTIME_TICK_FRQ_10K
                RUNTIME_TICK_FRQ_5K
                RUNTIME_TICK_FRQ_2K
                RUNTIME_TICK_FRQ_1K
**/
bool Runtime_Config(uint32_t tick_frq)
{
    uint32_t frq = RUNTIME_TICK_FRQ_20K;

    if (tick_frq == RUNTIME_TICK_FRQ_10K)
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

    SystemClock_Config();

    RunTime.module_state = Runtime_Module_Init;
    RunTime.base = RUNTIME_1SINUS / frq;
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
            RunTime.stop_callback();

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

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /** Supply configuration update enable
     */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    /** Configure the main internal regulator output voltage
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
    {
    }
    /** Macro to configure the PLL clock source
     */
    __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 5;
    RCC_OscInitStruct.PLL.PLLN = 128;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 8;
    RCC_OscInitStruct.PLL.PLLR = 8;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_UART4 | RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_UART8 | RCC_PERIPHCLK_UART5 | RCC_PERIPHCLK_SPI2;
    PeriphClkInitStruct.PLL3.PLL3M = 25;
    PeriphClkInitStruct.PLL3.PLL3N = 240;
    PeriphClkInitStruct.PLL3.PLL3P = 4;
    PeriphClkInitStruct.PLL3.PLL3Q = 3;
    PeriphClkInitStruct.PLL3.PLL3R = 2;
    PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_0;
    PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
    PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL3;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
}

static void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
