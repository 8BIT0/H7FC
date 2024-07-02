#ifndef __BSP_SPI_H
#define __BSP_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "Bsp_SPI_Port_Def.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_spi.h"

extern BspSpi_TypeDef BspSPI;

#ifdef __cplusplus
}
#endif

#endif
