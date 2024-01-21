#ifndef __KERNEL_STM32XXX_H
#define __KERNEL_STM32XXX_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32h7xx.h"

#define Kernel_DisableIRQ() __asm("cpsid i")
#define Kernel_EnableIRQ() __asm("cpsie i")

#endif
