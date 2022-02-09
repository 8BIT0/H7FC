#ifndef __DEBUG_UTIL_H
#define __DEBUG_UTIL_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc.h"

void assert(bool state);

#endif