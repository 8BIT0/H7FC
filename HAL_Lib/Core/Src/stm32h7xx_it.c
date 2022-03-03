#include "stm32h7xx_it.h"
#include "scheduler.h"
#include "kernel.h"
#include "runtime.h"

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
  if (Os_State() == Scheduler_ready)
  {
    Kernel_StkReg_Init();
  }
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  Runtime_Tick();
}
