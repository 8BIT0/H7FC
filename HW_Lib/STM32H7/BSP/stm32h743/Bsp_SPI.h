#ifndef __BSP_SPI_H
#define __BSP_SPI_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_spi.h"

#pragma pack(1)
typedef struct
{
    GPIO_TypeDef *port_mosi;
    GPIO_TypeDef *port_miso;
    GPIO_TypeDef *port_clk;

    uint32_t pin_mosi;
    uint32_t pin_miso;
    uint32_t pin_clk;

    uint32_t pin_Alternate;
} BspSPI_PinConfig_TypeDef;

typedef struct
{
    BspSPI_PinConfig_TypeDef Pin;
    SPI_TypeDef *Instance;
    uint32_t CLKPolarity;
    uint32_t CLKPhase;
    uint32_t BaudRatePrescaler;
} BspSPI_NorModeConfig_TypeDef;
#pragma pack()

typedef struct
{
    bool (*init)(BspSPI_NorModeConfig_TypeDef spi_cfg, SPI_HandleTypeDef *spi_instance);
    bool (*deinit)(BspSPI_NorModeConfig_TypeDef spi_cfg);
    bool (*trans)(SPI_HandleTypeDef *instance, uint8_t *tx, uint16_t size, uint16_t time_out);
    bool (*receive)(SPI_HandleTypeDef *instance, uint8_t *rx, uint16_t size, uint16_t time_out);
    bool (*trans_receive)(SPI_HandleTypeDef *instance, uint8_t *tx, uint8_t *rx, uint16_t size, uint16_t time_out);
    bool (*set_speed)(SPI_HandleTypeDef *instance, uint32_t speed);
} BspSpi_TypeDef;

extern BspSpi_TypeDef BspSPI;

#endif
