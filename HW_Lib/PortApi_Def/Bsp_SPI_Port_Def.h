#ifndef __BSP_SPI_PORT_DEF_H
#define __BSP_SPI_PORT_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define To_NormalSPI_Obj(x) (*((BspSPI_Config_TypeDef *)x))
#define To_NormalSPI_ObjPtr(x) ((BspSPI_Config_TypeDef *)x)
#define ToSPI_BusAPI(x) ((BspSpi_TypeDef *)x)

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

typedef enum
{
    BspSPI_Mode_Master = 0,
    BspSPI_Mode_Slave,
} BspSPI_WorkMode_List;

typedef struct
{
    BspSPI_PinConfig_TypeDef Pin;
    BspSPI_WorkMode_List work_mode;
    void *Instance;
    uint32_t CLKPolarity;
    uint32_t CLKPhase;
    uint32_t BaudRatePrescaler;
} BspSPI_Config_TypeDef;

typedef struct
{
    bool (*init)(BspSPI_Config_TypeDef spi_cfg, void *spi_instance);
    bool (*deinit)(BspSPI_Config_TypeDef spi_cfg);
    bool (*trans)(void *instance, uint8_t *tx, uint16_t size, uint16_t time_out);
    bool (*receive)(void *instance, uint8_t *rx, uint16_t size, uint16_t time_out);
    uint16_t (*trans_receive)(void *instance, uint8_t *tx, uint8_t *rx, uint16_t size, uint16_t time_out);
    bool (*set_speed)(void *instance, uint32_t speed);
} BspSpi_TypeDef;

#ifdef __cplusplus
}
#endif

#endif
