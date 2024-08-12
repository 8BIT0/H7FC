#ifndef __BSP_IIC_H
#define __BSP_IIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Bsp_IIC_Port_Def.h"

typedef enum
{
    BspIIC_Instance_I2C_1 = 0,
    BspIIC_Instance_I2C_2,
    BspIIC_Instance_I2C_3,
    BspIIC_Instance_I2C_Sum,
}BspIIC_Instance_List;

extern BspIIC_TypeDef BspIIC;

#ifdef __cplusplus
}
#endif

#endif
