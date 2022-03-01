#ifndef __KERNEL_H
#define __KERNEL_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "stm32h7xx.h"

bool Kernel_Init(void);
void Kernel_SetPendSV(void);
void Kernel_TriggerPendSV(void);

#endif
