#ifndef __BSP_IIC_H
#define __BSP_IIC_H

#include <stdbool.h>
#include <string.h>
#include <stdint.h>

typedef struct
{

}BspIICObj_TypeDef;

typedef struct
{
    bool (*init)(BspIICObj_TypeDef *obj);
    bool (*de_init)(BspIICObj_TypeDef *obj);
    uint16_t (*read)(BspIICObj_TypeDef *obj, uint8_t addr, uint8_t reg, uint8_t *p_data, uint16_t len);
    uint16_t (*write)(BspIICObj_TypeDef *obj, uint8_t addr, uint8_t reg, uint8_t *p_data, uint16_t len);
}BspIIC_TypeDef;

#endif
