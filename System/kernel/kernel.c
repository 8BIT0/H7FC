#include "kernel.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_pwr.h"
#include "system_cfg.h"

/* isolate kernel using stack with user stack  */
static uint8_t Kernel_Stack[KERNEL_STACK_SIZE] __attribute__((section(".kernel_stack_section")));

static bool KernelClock_Init(void);

bool Kernel_Init(void)
{
    // extern uint8_t tcm_code_start;
    // extern uint8_t tcm_code_end;
    // extern uint8_t tcm_code;

    // uint32_t *SourceAddr = (uint32_t *)FLASH_BANK1_BASE;
    // uint32_t *DestAddr = (uint32_t *)D1_DTCMRAM_BASE;
    // memcpy(DestAddr, SourceAddr, 0x400);
    // SCB->VTOR = D1_DTCMRAM_BASE;

    // disable interrupt at the first place
    Kernel_DisableIRQ();

    // clear kernel stack
    memset(Kernel_Stack, NULL, KERNEL_STACK_SIZE);

    SCB_EnableICache();
    SCB_EnableDCache();

    HAL_Init();

    // memcpy(&tcm_code_start, &tcm_code, (size_t)(&tcm_code_end - &tcm_code_start));

    return KernelClock_Init();
}

static bool KernelClock_Init(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /** Supply configuration update enable
     */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    /** Configure the main internal regulator output voltage
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

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
        __disable_irq();
        return false;
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
        __disable_irq();
        return false;
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
        __disable_irq();
        return false;
    }

    __enable_irq();
    return true;
}

__attribute__((naked)) void Kernel_SetPendSV(void)
{
    __ASM(".equ NVIC_SYSPRI14, 0xE000ED22");
    __ASM(".equ NVIC_PENDSV_PRI, 0xFF");

    __ASM("LDR      R0, =NVIC_SYSPRI14");
    __ASM("LDR      R1, =NVIC_PENDSV_PRI");
    __ASM("STRB     R1, [R0]");
    __ASM("BX       LR");
}

__attribute__((naked)) void Kernel_TriggerPendSV(void)
{
    __ASM(".equ NVIC_INT_CTRL, 0xE000ED04");
    __ASM(".equ NVIC_PENDSVSET, 0x10000000");

    __ASM("LDR      R0, =NVIC_INT_CTRL");
    __ASM("LDR      R1, =NVIC_PENDSVSET");
    __ASM("STR      R1, [R0]");
    __ASM("BX       LR");
}
