#ifndef __DEV_BMP280_H
#define __DEV_BMP280_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

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
} DevBMP280_ErrorCode_List;

typedef uint32_t (*DevBMP280_Get_Tick)(void);
typedef void (*DevBMP280_Delay_Ms)(uint32_t ms);
typedef uint16_t (*DevBMP280_BusCommu)(uint8_t *p_data, uint16_t len);
typedef uitn16_t (*DevBMP280_Trans)(uint8_t *p_tx, uint8_t *p_rx, uint16_t len);
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
