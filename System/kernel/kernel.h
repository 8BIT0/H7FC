#ifndef __KERNEL_H
#define __KERNEL_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "stm32h7xx.h"

#define Kernel_DisableIRQ() __asm("cpsid i")
#define Kernel_EnableIRQ() __asm("cpsie i")

bool Kernel_Init(void);
void Kernel_EnablePendSV(void);
void Kernel_TriggerPendSV(void);
void Kernel_LoadProcess(void);
void Kernel_EnterCritical(void);
void Kernel_ExitCritical(void);

#endif
