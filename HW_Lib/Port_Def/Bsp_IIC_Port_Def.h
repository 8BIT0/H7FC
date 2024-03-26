#ifndef __BSP_IIC_PORT_DEF_H
#define __BSP_IIC_PORT_DEF_H

#include <stdbool.h>
#include <string.h>
#include <stdint.h>

#define ToIIC_BusObj(x) ((BspIICObj_TypeDef *)x)
#define ToIIC_BusAPI(x) ((BspIIC_TypeDef *)x)

typedef struct
{
    void *port_sda;
    void *port_sck;

    uint32_t pin_sda;
    uint32_t pin_sck;

    uint32_t pin_Alternate;
}BspIIC_PinConfig_TypeDef;

typedef struct
{
    BspIIC_PinConfig_TypeDef *Pin;

    bool init;
    uint8_t instance_id;
    void *PeriphClkInitStruct;
    void *handle;
}BspIICObj_TypeDef;

typedef struct
{
    bool (*init)(BspIICObj_TypeDef *obj);
    bool (*de_init)(BspIICObj_TypeDef *obj);
    bool (*read)(BspIICObj_TypeDef *obj, uint16_t addr, uint16_t reg, uint8_t *p_data, uint16_t len);
    bool (*write)(BspIICObj_TypeDef *obj, uint16_t addr, uint16_t reg, uint8_t *p_data, uint16_t len);
}BspIIC_TypeDef;

void *BspIIC_Get_HandlePtr(uint8_t index);

#endif
