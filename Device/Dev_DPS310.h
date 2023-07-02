#ifndef __DEV_DPS310_H
#define __DEV_DPS310_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef bool (*DevDPS310_BusWrite)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *p_data, uint16_t len);
typedef bool (*DevDPS310_BusRead)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *p_data, uint16_t len);
typedef void (*DevDPS310_DelayMs)(uint32_t ms);
typedef void (*DevDPS310_WakeUp)(void);
typedef void (*DevDPS310_Sleep)(void);

#define DPS310_I2C_ADDR 0x76

#define DPS310_PSR_B2_REG 0x00u
#define DPS310_DSP_B1_REG 0x01u
#define DPS310_DSP_B0_REG 0x02u

#define DPS310_TMP_B2_REG 0x03u
#define DPS310_TMP_B1_REG 0x04u
#define DPS310_TMP_B0_REG 0x05u

#define DPS310_PRS_CFG_REG 0x06u

#define DPS310_PRS_CFG_PM_RATE_MASK 0x70u

#define DPS310_CFG_RATE_1_MEAS 0x00u
#define DPS310_CFG_RATE_2_MEAS 0x10u
#define DPS310_CFG_RATE_4_MEAS 0x20u
#define DPS310_CFG_RATE_8_MEAS 0x30u
#define DPS310_CFG_RATE_16_MEAS 0x40u
#define DPS310_CFG_RATE_32_MEAS 0x50u   //101 - 32 measurements pr. sec
#define DPS310_CFG_RATE_64_MEAS 0x60u
#define DPS310_CFG_RATE_128_MEAS 0x70u

#define DPS310_PRS_CFG_PM_PRC_SINGLE 0x00u  // low precision
#define DPS310_PRS_CFG_PM_PRC_2_TIMES 0x01u  // low power
#define DPS310_PRS_CFG_PM_PRC_4_TIMES 0x02u
#define DPS310_PRS_CFG_PM_PRC_8_TIMES 0x03u
#define DPS310_PRS_CFG_PM_PRC_16_TIMES 0x04u // standard
#define DPS310_PRS_CFG_PM_PRC_32_TIMES 0x05u
#define DPS310_PRS_CFG_PM_PRC_64_TIMES 0x06u // high precision
#define DPS310_PRS_CFG_PM_PRC_128_TIMES 0x07u

#define DPS310_TMP_CFG_REG 0x07u

#define DPS310_TMP_CFG_REG_TMP_EXT_INTERNAL 0x00u  // in ASIC
#define DPS310_TMP_CFG_REG_TMP_EXT_EXTERNAL 0x80u  // in pressure sensor MEMS element

#define DPS310_TMP_CFG_TMP_RATE_MASK 0x70u

#define DPS310_TMP_CFG_TMP_PRC_SINGLE 0x00u  // default - measurment time 3.6 ms
#define DPS310_TMP_CFG_TMP_PRC_2_TIMES 0x01u
#define DPS310_TMP_CFG_TMP_PRC_4_TIMES 0x02u
#define DPS310_TMP_CFG_TMP_PRC_8_TIMES 0x03u
#define DPS310_TMP_CFG_TMP_PRC_16_TIMES 0x04u
#define DPS310_TMP_CFG_TMP_PRC_32_TIMES 0x05u
#define DPS310_TMP_CFG_TMP_PRC_64_TIMES 0x06u
#define DPS310_TMP_CFG_TMP_PRC_128_TIMES 0x07u

#define DPS310_MEAS_CFG_REG 0x08u

#define DPS310_MEAS_CFG_COEF_RDY_AVAILABLE 0x80u
#define DPS310_MEAS_CFG_SENSOR_RDY_COMPLETE 0x40u
#define DPS310_MEAS_CFG_TMP_RDY 0x20u
#define DPS310_MEAS_CFG_PRS_RDY 0x10u

#define DPS310_MEAS_CFG_MEAS_CTRL_MASK 0x07u
#define DPS310_MEAS_CFG_MEAS_CTRL_IDLE 0x00u
#define DPS310_MEAS_CFG_MEAS_CTRL_PRS 0x01u
#define DPS310_MEAS_CFG_MEAS_CTRL_TMP 0x02u
#define DPS310_MEAS_CFG_MEAS_CTRL_CONTINUOUS_PRS 0x05u
#define DPS310_MEAS_CFG_MEAS_CTRL_CONTINUOUS_TMP 0x06u
#define DPS310_MEAS_CFG_MEAS_CTRL_CONTINUOUS_PRS_TMP 0x07u

#define DPS310_CFG_REG_REG 0x09u
#define DPS310_CFG_RET_INT_HL 0x80u
#define DPS310_CFG_RET_INT_SEL 0x70u
#define DPS310_CFG_RET_TMP_SHIFT_EN 0x08u
#define DPS310_CFG_RET_PRS_SHIFT_EN 0x04u
#define DPS310_CFG_RET_FIFO_EN 0x02u
#define DPS310_CFG_RET_TMP_SPI_MODE 0x01u

#define DPS310_INT_STS_REG 0x0Au
#define DPS310_INT_STS_INT_FIFO_FULL_MASK 0x04u
#define DPS310_INT_STS_INT_TMP_MASK 0x02u
#define DPS310_INT_STS_INT_PRS_MASK 0x01u

#define DPS310_FIFO_STS_REG 0x0Bu
#define DPS310_FIFO_STS_FIFO_FULL_MASK 0x02u
#define DPS310_FIFO_STS_FIFO_EMPTY_MASK 0x01u

#define DPS310_RESET_REG 0x0Cu
#define DPS310_RESET_FIFO_FLUSH 0x80u
#define DPS310_RESET_SOFT_RST_VALUE 0x09u

#define DPS310_PRODUCT_ID_REG 0x0Du
#define DPS310_PRODUCT_ID_VALUE 0x10u
#define DPS310_PRODUCT_ID_REV_ID_MASK 0xF0u
#define DPS310_PRODUCT_ID_PROD_ID_MASK 0x0Fu

#define DPS310_COEF_REG 0x10u

#define DPS310_TMP_COEF_SRCE 0x28u

#define DPS310_TMP_COEF_SRCE_MASK 0x80u
#define DPS310_TMP_COEF_SRCE_EXTERNAL 0x80u
#define DPS310_TMP_COEF_SRCE_INTERNAL 0x00u

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
    DevDPS310_Error_PressureInit,
    DevDPS310_Error_TempratureInit,
    DevDPS310_Error_CaliCoefs,
}DevDPS310_ErrorList;

typedef struct {
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
} DevDPS310_Cali_Coefs_TypeDef;

typedef struct
{
    uint8_t DevAddr;
    uint8_t ProdID;
    uint32_t update_time;
    uint32_t pres_factory_scale;
    uint32_t temp_factory_scale;
    float scaled_pres;
    int16_t pres;

    DevDPS310_BusWrite bus_tx;
    DevDPS310_BusRead  bus_rx;
    DevDPS310_DelayMs  bus_delay;
    DevDPS310_WakeUp   bus_wakeup;
    DevDPS310_Sleep    bus_sleep;

    DevDPS310_Cali_Coefs_TypeDef cali_coefs;

    bool ready;
    DevDPS310_ErrorList error;
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

