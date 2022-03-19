#include "stm32h7xx_it.h"
#include "kernel.h"
#include "runtime.h"
#include "scheduler.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

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

  /* Os relay on */
  Runtime_Tick();

  /* periph relay on */
  if (time_base / REAL_1MS)
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
