#ifndef __BSP_QSPI_PORT_DEF_H
#define __BSP_QSPI_PORT_DEF_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct
{
    void *port_clk;
    void *port_ncs;
    void *port_io0;
    void *port_io1;
    void *port_io2;
    void *port_io3;

    uint32_t pin_clk;
    uint32_t pin_ncs;
    uint32_t pin_io0;
    uint32_t pin_io1;
    uint32_t pin_io2;
    uint32_t pin_io3;

    uint32_t alt_clk;
    uint32_t alt_ncs;
    uint32_t alt_io0;
    uint32_t alt_io1;
    uint32_t alt_io2;
    uint32_t alt_io3;
} BspQSPI_PinConfig_TypeDef;

typedef struct
{
    bool init_state;
    void *p_qspi;
    BspQSPI_PinConfig_TypeDef pin;
} BspQSPI_Config_TypeDef;

typedef struct
{
    bool (*init)(BspQSPI_Config_TypeDef *obj);
    bool (*tx)(BspQSPI_Config_TypeDef *obj, uint32_t addr, uint32_t cmd, uint8_t *p_data, uint32_t len);
    bool (*rx)(BspQSPI_Config_TypeDef *obj, uint32_t addr, uint32_t cmd, uint8_t *p_data, uint32_t len);
    bool (*cmd)(BspQSPI_Config_TypeDef *obj, uint32_t mode, uint32_t dummy_cyc, uint32_t nb_data, uint32_t cmd);
    bool (*polling)(BspQSPI_Config_TypeDef *obj);
    bool (*memmap)(BspQSPI_Config_TypeDef *obj);    /* for fast read */
} BspQSpi_TypeDef;

#endif