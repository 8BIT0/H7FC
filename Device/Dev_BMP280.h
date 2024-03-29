#ifndef __DEV_BMP280_H
#define __DEV_BMP280_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define DevBMP280_ADDR                  0x76

#define DevBMP280_SLEEP_MODE            0x00
#define DevBMP280_FORCED_MODE           0x01
#define DevBMP280_NORMAL_MODE           0x03

#define DevBMP280_STDBY_HALF_MS         0x00
#define DevBMP280_STDBY_62_AND_HALF_MS  0x01
#define DevBMP280_STDBY_125_MS          0x02
#define DevBMP280_STDBY_250_MS          0x03
#define DevBMP280_STDBY_500_MS          0x04
#define DevBMP280_STDBY_1_SEC           0x05
#define DevBMP280_STDBY_2_SEC           0x06
#define DevBMP280_STDBY_4_SEC           0x07

#define DevBMP280_TEMP_STOP             0x00
#define DevBMP280_TEMP_16_BIT           0x01
#define DevBMP280_TEMP_17_BIT           0x02
#define DevBMP280_TEMP_18_BIT           0x03
#define DevBMP280_TEMP_19_BIT           0x04
#define DevBMP280_TEMP_20_BIT           0x05

#define DevBMP280_PRESS_STOP            0x00
#define DevBMP280_PRESS_16_BIT          0x01
#define DevBMP280_PRESS_17_BIT          0x02
#define DevBMP280_PRESS_18_BIT          0x03
#define DevBMP280_PRESS_19_BIT          0x04
#define DevBMP280_PRESS_20_BIT          0x05

#define DevBMP280_POWER_MODE_MASK       0xFC
#define DevBMP280_STDBY_TIME_MASK       0x1F
#define DevBMP280_TEMP_MASK             0x1F
#define DevBMP280_PRESS_MASK            0xE3

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
