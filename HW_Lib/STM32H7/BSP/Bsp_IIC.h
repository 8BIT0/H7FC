#ifndef __BSP_IIC_H
#define __BSP_IIC_H

#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "Bsp_IIC_Port_Def.h"
#include "Bsp_GPIO.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_i2c.h"

#define I2C_HandleType_Size sizeof(I2C_HandleTypeDef)
#define I2C_PeriphCLKInitType_Size sizeof(RCC_PeriphCLKInitTypeDef)

typedef enum
{
    BspIIC_Instance_I2C_1 = 0,
    BspIIC_Instance_I2C_2,
    BspIIC_Instance_I2C_3,
    BspIIC_Instance_I2C_4,
    BspIIC_Instance_I2C_Sum,
}BspIIC_Instance_List;

extern BspIIC_TypeDef BspIIC;

#endif
