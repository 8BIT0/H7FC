#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "Bsp_GPIO_Port_Def.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_gpio.h"

#define GPIO_EXTI_SUM 16

extern BspGPIO_TypeDef BspGPIO;

#endif
