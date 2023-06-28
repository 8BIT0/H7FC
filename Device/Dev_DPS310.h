#ifndef __DEV_DPS310_H
#define __DEV_DPS310_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef bool (*DevDPS310_BusWrite)(uint8_t dev_addr, uint8_t *p_data, uint16_t len, bool ack);
typedef bool (*DevDPS310_BusRead)(uint8_t dev_addr, uint8_t *p_data, uint16_t len, bool ack);

#define NUM_OF_COMMON_REGMASKS 16

#define DEV_DPS310_PROD_ID          0x00
#define DEV_DPS310_SPI_WRITE_CMD    0x00U
#define DEV_DPS310_SPI_READ_CMD     0x80U
#define DEV_DPS310_SPI_RW_MASK      0x80U
#define DEV_DPS310_SPI_MAX_FREQ     1000000U
#define DEV_DPS310_OSR_SE           3U

// DPS310 has 10 milliseconds of spare time for each synchronous measurement / per second for asynchronous measurements
// this is for error prevention on friday-afternoon-products :D
// you can set it to 0 if you dare, but there is no warranty that it will still work
#define DEV_DPS310_BUSYTIME_FAILSAFE    10U
#define DEV_DPS310_MAX_BUSYTIME         ((1000U - DEV_DPS310_BUSYTIME_FAILSAFE) * DEV_DPS_BUSYTIME_SCALING)

#define DEV_DPS310_REG_ADR_SPI3W        0x09U
#define DEV_DPS310_REG_CONTENT_SPI3W    0x01U

#define DEV_DPS310_FIFO_SIZE            32
#define DEV_DPS310_STD_SLAVE_ADDRESS    0x77U
#define DEV_DPS310_RESULT_BLOCK_LENGTH  3

#define DEV_DPS310_MEASUREMENT_RATE_1   0
#define DEV_DPS310_MEASUREMENT_RATE_2   1
#define DEV_DPS310_MEASUREMENT_RATE_4   2
#define DEV_DPS310_MEASUREMENT_RATE_8   3
#define DEV_DPS310_MEASUREMENT_RATE_16  4
#define DEV_DPS310_MEASUREMENT_RATE_32  5
#define DEV_DPS310_MEASUREMENT_RATE_64  6
#define DEV_DPS310_MEASUREMENT_RATE_128 7

#define DEV_DPS310_OVERSAMPLING_RATE_1      DEV_DPS310_MEASUREMENT_RATE_1
#define DEV_DPS310_OVERSAMPLING_RATE_2      DEV_DPS310_MEASUREMENT_RATE_2
#define DEV_DPS310_OVERSAMPLING_RATE_4      DEV_DPS310_MEASUREMENT_RATE_4
#define DEV_DPS310_OVERSAMPLING_RATE_8      DEV_DPS310_MEASUREMENT_RATE_8
#define DEV_DPS310_OVERSAMPLING_RATE_16     DEV_DPS310_MEASUREMENT_RATE_16
#define DEV_DPS310_OVERSAMPLING_RATE_32     DEV_DPS310_MEASUREMENT_RATE_32
#define DEV_DPS310_OVERSAMPLING_RATE_64     DEV_DPS310_MEASUREMENT_RATE_64
#define DEV_DPS310_OVERSAMPLING_RATE_128    DEV_DPS310_MEASUREMENT_RATE_128

//we use 0.1 ms units for time calculations, so 10 units are one millisecond
#define DEV_DPS310_BUSYTIME_SCALING         10U
#define DEV_DPS310_NUM_OF_SCAL_FACTS        8

typedef struct
{
    uint8_t addr;
    uint8_t mask;
    uint8_t shift;
}DevDPS310_RegMask_TypeDef;

typedef struct
{
    uint8_t addr;
    uint8_t len;
}DevDPS310_RegBlock_TypeDef;

typedef enum
{
    DevDPS310_NO_INTR = 0,
    DevDPS310_PRS_INTR,
    DevDPS310_TEMP_INTR,
    DevDPS310_BOTH_INTR,
    DevDPS310_FIFO_FULL_INTR,
}DevDPS310_IntSource_TypeList;

typedef enum
{
    DevDPS310_Error_None = 0,
    DevDPS310_Error_Busy,
    DevDPS310_Error_BadID,
}DevDPS310_ErrorList;

typedef struct
{
    uint8_t DevAddr;
    float factory_scale;
    float scaled_pres;
    int16_t pres;

    DevDPS310_BusWrite bus_tx;
    DevDPS310_BusRead bus_rx;

    bool ready;
}DevDPS310Obj_TypeDef;

typedef struct
{
    bool (*pre_init)(DevDPS310Obj_TypeDef *obj, DevDPS310_BusWrite write, DevDPS310_BusRead read);
    bool (*init)(DevDPS310Obj_TypeDef *obj, uint8_t dev_addr);
    bool (*sample)(DevDPS310Obj_TypeDef *obj);
    bool (*ready)(DevDPS310Obj_TypeDef *obj);
    float (*get_scaled_pres)(DevDPS310Obj_TypeDef *obj);
    int16_t (*get_none_scaled_pres)(DevDPS310Obj_TypeDef *obj);
}DevDPS310_TypeDef;

#endif

