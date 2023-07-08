#ifndef __BSP_IIC_H
#define __BSP_IIC_H

#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "Bsp_GPIO.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_i2c.h"

typedef enum
{
    BspIIC_Instance_I2C_1 = 0,
    BspIIC_Instance_I2C_2,
    BspIIC_Instance_I2C_3,
    BspIIC_Instance_I2C_4,
}BspIIC_Instance_List;

typedef struct
{
    BspIIC_Instance_List instance_id;
    I2C_HandleTypeDef handle;
}BspIICObj_TypeDef;

typedef struct
{
    bool (*init)(BspIICObj_TypeDef *obj);
    bool (*de_init)(BspIICObj_TypeDef *obj);
    uint16_t (*read)(BspIICObj_TypeDef *obj, uint8_t addr, uint8_t reg, uint8_t *p_data, uint16_t len);
    uint16_t (*write)(BspIICObj_TypeDef *obj, uint8_t addr, uint8_t reg, uint8_t *p_data, uint16_t len);
}BspIIC_TypeDef;

#endif
