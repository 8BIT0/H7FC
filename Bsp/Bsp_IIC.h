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
    BspIIC_Instance_I2C_Sum,
}BspIIC_Instance_List;

typedef struct
{
    GPIO_TypeDef *port_sda;
    GPIO_TypeDef *port_sck;

    uint32_t pin_sda;
    uint32_t pin_sck;

    uint32_t pin_Alternate;
}BspIIC_PinConfig_TypeDef;

typedef struct
{
    BspIIC_PinConfig_TypeDef *Pin;

    bool init;
    BspIIC_Instance_List instance_id;
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
    I2C_HandleTypeDef handle;
}BspIICObj_TypeDef;

typedef struct
{
    bool (*init)(BspIICObj_TypeDef *obj);
    bool (*de_init)(BspIICObj_TypeDef *obj);
    bool (*read)(BspIICObj_TypeDef *obj, uint16_t addr, uint16_t reg, uint8_t *p_data, uint16_t len);
    bool (*write)(BspIICObj_TypeDef *obj, uint16_t addr, uint16_t reg, uint8_t *p_data, uint16_t len);
}BspIIC_TypeDef;

I2C_HandleTypeDef *BspIIC_Get_HandlePtr(BspIIC_Instance_List index);

extern BspIIC_TypeDef BspIIC;

#endif
