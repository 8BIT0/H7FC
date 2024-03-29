#ifndef __DEV_BMP280_H
#define __DEV_BMP280_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define BMP280_REG_NVM_PAR_T1_L        0x88        /**< NVM PAR T1 low register */
#define BMP280_REG_NVM_PAR_T1_H        0x89        /**< NVM PAR T1 high register */
#define BMP280_REG_NVM_PAR_T2_L        0x8A        /**< NVM PAR T2 low register */
#define BMP280_REG_NVM_PAR_T2_H        0x8B        /**< NVM PAR T2 high register */
#define BMP280_REG_NVM_PAR_T3_L        0x8C        /**< NVM PAR T3 low register */
#define BMP280_REG_NVM_PAR_T3_H        0x8D        /**< NVM PAR T3 high register */
#define BMP280_REG_NVM_PAR_P1_L        0x8E        /**< NVM PAR P1 low register */
#define BMP280_REG_NVM_PAR_P1_H        0x8F        /**< NVM PAR P1 high register */
#define BMP280_REG_NVM_PAR_P2_L        0x90        /**< NVM PAR P2 low register */
#define BMP280_REG_NVM_PAR_P2_H        0x91        /**< NVM PAR P2 high register */
#define BMP280_REG_NVM_PAR_P3_L        0x92        /**< NVM PAR P3 low register */
#define BMP280_REG_NVM_PAR_P3_H        0x93        /**< NVM PAR P3 high register */
#define BMP280_REG_NVM_PAR_P4_L        0x94        /**< NVM PAR P4 low register */
#define BMP280_REG_NVM_PAR_P4_H        0x95        /**< NVM PAR P4 high register */
#define BMP280_REG_NVM_PAR_P5_L        0x96        /**< NVM PAR P5 low register */
#define BMP280_REG_NVM_PAR_P5_H        0x97        /**< NVM PAR P5 high register */
#define BMP280_REG_NVM_PAR_P6_L        0x98        /**< NVM PAR P6 low register */
#define BMP280_REG_NVM_PAR_P6_H        0x99        /**< NVM PAR P6 high register */
#define BMP280_REG_NVM_PAR_P7_L        0x9A        /**< NVM PAR P7 low register */
#define BMP280_REG_NVM_PAR_P7_H        0x9B        /**< NVM PAR P7 high register */
#define BMP280_REG_NVM_PAR_P8_L        0x9C        /**< NVM PAR P8 low register */
#define BMP280_REG_NVM_PAR_P8_H        0x9D        /**< NVM PAR P8 high register */
#define BMP280_REG_NVM_PAR_P9_L        0x9E        /**< NVM PAR P9 low register */
#define BMP280_REG_NVM_PAR_P9_H        0x9F        /**< NVM PAR P9 high register */
#define BMP280_REG_TEMP_XLSB           0xFC        /**< temp xlsb register */
#define BMP280_REG_TEMP_LSB            0xFB        /**< temp lsb register */
#define BMP280_REG_TEMP_MSB            0xFA        /**< temp msb register */
#define BMP280_REG_PRESS_XLSB          0xF9        /**< press xlsb register */
#define BMP280_REG_PRESS_LSB           0xF8        /**< press lsb register */
#define BMP280_REG_PRESS_MSB           0xF7        /**< press msb register */
#define BMP280_REG_CONFIG              0xF5        /**< config register */
#define BMP280_REG_CTRL_MEAS           0xF4        /**< ctrl meas register */
#define BMP280_REG_STATUS              0xF3        /**< status register */
#define BMP280_REG_RESET               0xE0        /**< soft reset register */
#define BMP280_REG_ID                  0xD0        /**< chip id register */
#define BMP280_DEVICE_ID               0x58        /**< chip id */

typedef enum
{
    DevBMP280_Bus_SPI = 0,
    DevBMP280_Bus_IIC,
} DevBMP280_BusType_List;

typedef enum
{
    DevBMP280_Error_None = 0,
    DevBMP280_Para_Error,
    DevBMP280_Init_Error,
    DevBMP280_ID_Error,
} DevBMP280_ErrorCode_List;

typedef uint32_t (*DevBMP280_Get_Tick)(void);
typedef void (*DevBMP280_Delay_Ms)(uint32_t ms);
typedef uint16_t (*DevBMP280_BusCommu)(uint8_t *p_data, uint16_t len);
typedef uint16_t (*DevBMP280_Trans)(uint8_t *p_tx, uint8_t *p_rx, uint16_t len);
typedef void (*DevBMP280_CS_Ctl)(bool state); /* true -> cs high / false -> cs low */

typedef struct
{
    DevBMP280_ErrorCode_List ErrorCode;
    DevBMP280_BusType_List Bus;

    DevBMP280_Get_Tick get_tick;
    DevBMP280_Delay_Ms delay_ms;
    DevBMP280_BusCommu send;
    DevBMP280_BusCommu recv;
    DevBMP280_Trans trans;
    DevBMP280_CS_Ctl cs_ctl;

} DevBMP280Obj_TypeDef;

typedef struct
{
    bool (*init)(DevBMP280Obj_TypeDef *obj);
} DevBMP280_TypeDef;

extern DevBMP280_TypeDef DevBMP280;

#endif
