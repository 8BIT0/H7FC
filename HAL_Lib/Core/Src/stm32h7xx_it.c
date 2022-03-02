#include "stm32h7xx_it.h"
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

uint32_t msp = 0;
void SVC_Handler(void)
{
  __asm volatile("MRS %0, msp"
                 : "=r"(msp));
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
