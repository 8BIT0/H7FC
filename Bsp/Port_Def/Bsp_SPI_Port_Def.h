#ifndef __BSP_SPI_PORT_DEF_H
#define __BSP_SPI_PORT_DEF_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    void *port_mosi;
    void *port_miso;
    void *port_clk;

    uint32_t pin_mosi;
    uint32_t pin_miso;
    uint32_t pin_clk;

    uint32_t pin_Alternate;
} BspSPI_PinConfig_TypeDef;

typedef struct
{
    BspSPI_PinConfig_TypeDef Pin;
    void *Instance;
    uint32_t CLKPolarity;
    uint32_t CLKPhase;
    uint32_t BaudRatePrescaler;
} BspSPI_NorModeConfig_TypeDef;

typedef struct
{
    bool (*init)(BspSPI_NorModeConfig_TypeDef spi_cfg, void *spi_instance);
    bool (*deinit)(BspSPI_NorModeConfig_TypeDef spi_cfg);
    bool (*trans)(void *instance, uint8_t *tx, uint16_t size, uint16_t time_out);
    bool (*receive)(void *instance, uint8_t *rx, uint16_t size, uint16_t time_out);
    bool (*trans_receive)(void *instance, uint8_t *tx, uint8_t *rx, uint16_t size, uint16_t time_out);
    bool (*set_speed)(void *instance, uint32_t speed);
} BspSpi_TypeDef;

#endif
