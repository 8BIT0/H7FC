#include "stm32h7xx_it.h"
#include "stm32h7xx_hal.h"
#include "kernel.h"
#include "runtime.h"
#include "scheduler.h"
#include "stm32h7xx_hal_gpio.h"
#include "IO_Definition.h"
#include "debug_util.h"
#include "DIskIO.h"
#include "DataPipe.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern DevCard_Obj_TypeDef DevTFCard_Obj;
extern DMA_HandleTypeDef DataPipe_DMA;

void NMI_Handler(void)
{
  while (1)
  {
  }
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
  Runtime_Start();
  Os_LoadFirstTask();
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
  Os_SwitchContext();
}

void SysTick_Handler(void)
{
  static uint32_t time_base = 0;

  DebugPin.ctl(Debug_PB3, true);

  /* Os relay on */
  Runtime_Tick();

  DebugPin.ctl(Debug_PB3, false);

  /* periph relay on */
  if (time_base == REAL_1MS - Runtime_GetTickBase())
  {
    time_base = 0;
    HAL_IncTick();
  }
  else
  {
    time_base += Runtime_GetTickBase();
  }
}

void OTG_FS_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}

void EXTI0_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

void EXTI1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

void EXTI2_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

void EXTI3_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}

void EXTI4_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
}

void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
}

void SDMMC1_IRQHandler(void)
{
  HAL_SD_IRQHandler(&DevTFCard_Obj.SDMMC_Obj.hdl);
}

void MDMA_IRQHandler(void)
{
  HAL_MDMA_IRQHandler(&DevTFCard_Obj.SDMMC_Obj.mdma);
}

void DMA2_Stream7_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&DataPipe_DMA);
}

/* unmodify down below */
void DMA1_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_uart7_rx);
}

void DMA1_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_uart7_tx);
}

void DMA1_Stream2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
}

void DMA1_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
}

void DMA1_Stream4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
}

void DMA1_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_uart4_tx);
}

void UART4_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart4);
}

void USART6_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart6);
}

void UART7_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart7);
}

