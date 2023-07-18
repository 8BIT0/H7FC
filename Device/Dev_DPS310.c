#include "Dev_DPS310.h"

#define DPS310_WAIT_REG_TIMEOUT 100

#define DPS310_SAMPLE_RATE DPS310_CFG_RATE_32_MEAS
#define DPS310_PROC_TIME   DPS310_PRS_CFG_PM_PRC_16_TIMES
#define DPS310_MAX_SAMPLE_RATE 10 /* unit:ms period 10ms 100Hz */
#define DPS310_MIN_SAMPLE_RATE 25 /* unit:ms period 25ms 40Hz  */

/* internal function */
static bool DevDPS310_WriteByteToReg(DevDPS310Obj_TypeDef *obj, uint8_t reg_addr, uint8_t data);
static bool DevDPS310_WaitRegValue(DevDPS310Obj_TypeDef *obj, uint8_t reg_addr, uint8_t reg_value, uint8_t mask);
static bool DevDPS310_Temperature_Sensor(DevDPS310Obj_TypeDef *obj, uint8_t *p_sensor);
static bool DevDPS310_Get_ProdID(DevDPS310Obj_TypeDef *obj);
static uint32_t DevDPS310_GetScale(DevDPS310Obj_TypeDef *obj, uint8_t rate);
static bool DevDPS310_WakeUp(DevDPS310Obj_TypeDef *obj, uint8_t mode);
static bool DevDPS310_Sleep(DevDPS310Obj_TypeDef *obj);
static bool DevDPS310_Reset(DevDPS310Obj_TypeDef *obj);
static bool DevDPS310_Configure_Temperature(DevDPS310Obj_TypeDef *obj, uint8_t data);
static int32_t DevDPS310_GetTwoComplementOf(uint32_t value, uint8_t length);
static bool DevDPS310_Get_Cali_Coefs(DevDPS310Obj_TypeDef *obj);

/* external function */
static bool DevDPS310_Init(DevDPS310Obj_TypeDef *obj);

/* external object */
DevDPS310_TypeDef DevDPS310 = {
    .init = DevDPS310_Init,
};

static bool DevDPS310_Init(DevDPS310Obj_TypeDef *obj)
{
    if(obj  && obj->bus_delay)
    {
        obj->error = DevDPS310_Error_None;

        if(!DevDPS310_Get_ProdID(obj))
        {
            obj->error = DevDPS310_Error_BadID;
            return false;
        }

        DevDPS310_Reset(obj);
        obj->bus_delay(50);

        /* get calibrate data */
        if(!DevDPS310_Get_Cali_Coefs(obj))
        {
            obj->error = DevDPS310_Error_CaliCoefs;
            return false;
        }

        /* configure pressure sensor */
        if(DevDPS310_WriteByteToReg(obj, DPS310_PRS_CFG_REG, DPS310_SAMPLE_RATE | DPS310_PROC_TIME))
        {
            obj->pres_factory_scale = DevDPS310_GetScale(obj, DPS310_SAMPLE_RATE);
        }
        else
        {
            /* presesure init error */
            obj->error = DevDPS310_Error_PressureInit;
            return false;
        }

        /* configure temprature sensor */
        if(DevDPS310_Configure_Temperature(obj, DPS310_SAMPLE_RATE | DPS310_PROC_TIME))
        {
            obj->temp_factory_scale = DevDPS310_GetScale(obj, DPS310_SAMPLE_RATE);
        }
        else
        {
            /* temprature init error */
            obj->error = DevDPS310_Error_TempratureInit;
            return false;
        }

        obj->DevAddr = DPS310_I2C_ADDR;

        return true;
    }

    return false;
}

static uint32_t DevDPS310_GetScale(DevDPS310Obj_TypeDef *obj, uint8_t rate)
{
    if(obj)
    {
        switch (rate) {
            case DPS310_CFG_RATE_1_MEAS:
                obj->factory_scale = 524288.0f;
                return 524288;
                
            case DPS310_CFG_RATE_2_MEAS:
                obj->factory_scale = 1572864.0f;
                return 1572864;

            case DPS310_CFG_RATE_4_MEAS:
                obj->factory_scale = 3670016.0f;
                return 3670016;

            case DPS310_CFG_RATE_8_MEAS:
                obj->factory_scale = 7864320.0f;
                return 7864320;

            case DPS310_CFG_RATE_16_MEAS:
                obj->factory_scale = 253952.0f;
                return 253952;

            case DPS310_CFG_RATE_32_MEAS:
                obj->factory_scale = 516096.0f;
                return 516096;

            case DPS310_CFG_RATE_64_MEAS:
                obj->factory_scale = 1040384.0f;
                return 1040384;

            case DPS310_CFG_RATE_128_MEAS:
                obj->factory_scale = 2088960.0f;
                return 2088960;

            default:
                return 0;
        }
    }

    return 0;
}

static bool DevDPS310_WakeUp(DevDPS310Obj_TypeDef *obj, uint8_t mode)
{
    if(obj)
        return DevDPS310_WriteByteToReg(obj, DPS310_MEAS_CFG_REG, mode);

    return false;
}

static bool DevDPS310_Sleep(DevDPS310Obj_TypeDef *obj)
{
    if(obj)
        return DevDPS310_WriteByteToReg(obj, DPS310_MEAS_CFG_REG, DPS310_MEAS_CFG_MEAS_CTRL_IDLE);

    return false;
}

static bool DevDPS310_Reset(DevDPS310Obj_TypeDef *obj)
{
    if(obj)
        return DevDPS310_WriteByteToReg(obj,DPS310_RESET_REG, DPS310_RESET_SOFT_RST_VALUE);

    return false;
}

static bool DevDPS310_Get_ProdID(DevDPS310Obj_TypeDef *obj)
{
    uint8_t data_tmp = 0;

    if(obj && obj->bus_rx && obj->bus_rx(obj->DevAddr, DPS310_PRODUCT_ID_REG, &data_tmp, 1))
    {
        obj->ProdID = data_tmp;
        if(data_tmp != DPS310_PRODUCT_ID_VALUE)
            return false;
        
        return true;
    }

    return false;
}

static bool DevDPS310_Configure_Temperature(DevDPS310Obj_TypeDef *obj, uint8_t data)
{
    int16_t ret;
    uint8_t temperature_sensor = DPS310_TMP_CFG_REG_TMP_EXT_EXTERNAL;

    if(obj)
    {
        if(DevDPS310_Temperature_Sensor(obj, &temperature_sensor))
        {
            data |= temperature_sensor;
        }
        else
            return false;

        return DevDPS310_WriteByteToReg(obj, DPS310_TMP_CFG_REG, data);
    }

    return false;
}

static bool DevDPS310_WriteByteToReg(DevDPS310Obj_TypeDef *obj, uint8_t reg_addr, uint8_t data)
{
    if(obj && obj->bus_tx)
    {
        if(!obj->bus_tx(obj->DevAddr, reg_addr, &data, 1))
            return false;
    }

    return true;
}

static int32_t DevDPS310_GetTwoComplementOf(uint32_t value, uint8_t length)
{
    int32_t ret = value;
    bool b_is_negative = value & (1u << (length - 1u));

    if (b_is_negative)
    {
        ret -= ((uint32_t) 1 << length);
    }

    return ret;
}

static bool DevDPS310_Temperature_Sensor(DevDPS310Obj_TypeDef *obj, uint8_t *p_sensor)
{
    uint8_t read = 0;

    if(obj && obj->bus_rx && obj->bus_rx(obj->DevAddr, DPS310_TMP_COEF_SRCE, &read, 1))
    {
        read &= DPS310_TMP_COEF_SRCE_MASK;

        if (read)
        {
            *p_sensor = DPS310_TMP_CFG_REG_TMP_EXT_EXTERNAL;
        }
        else
            *p_sensor = DPS310_TMP_CFG_REG_TMP_EXT_INTERNAL;

        return true;
    }

    return false;
}

static bool DevDPS310_WaitRegValue(DevDPS310Obj_TypeDef *obj, uint8_t reg_addr, uint8_t reg_value, uint8_t mask)
{
    uint8_t read_out = 0;
    uint16_t attempts = 0;

    if(obj && obj->bus_rx && obj->bus_delay)
    {
        while (attempts < DPS310_WAIT_REG_TIMEOUT)
        {
            attempts++;

            if(!obj->bus_rx(obj->DevAddr, reg_addr, &read_out, 1))
                return false;

            if ((read_out & mask) == reg_value)
                return true;

            obj->bus_delay(10);
        }

        if (attempts == DPS310_WAIT_REG_TIMEOUT)
            return false;
    }

    return true;
}

static bool DevDPS310_Get_Cali_Coefs(DevDPS310Obj_TypeDef *obj)
{
    uint8_t buff[18] = {0};

    if(obj &&
       obj->bus_rx && 
       DevDPS310_WaitRegValue(obj, 
                              DPS310_MEAS_CFG_REG,
                              DPS310_MEAS_CFG_COEF_RDY_AVAILABLE,
                              DPS310_MEAS_CFG_COEF_RDY_AVAILABLE) &&
       obj->bus_rx(obj->DevAddr, DPS310_COEF_REG, buff, 18))
    {
        obj->cali_coefs.c0  = DevDPS310_GetTwoComplementOf(((uint16_t) buff[0] << 4u) | (((uint16_t) buff[1] >> 4u) & 0x0Fu), 12);
        obj->cali_coefs.c1  = DevDPS310_GetTwoComplementOf(((((uint16_t) buff[1] & 0x0Fu) << 8u) | (uint16_t) buff[2]), 12);
        obj->cali_coefs.c00 = DevDPS310_GetTwoComplementOf(((uint32_t) buff[3] << 12u) | ((uint32_t) buff[4] << 4u) | (((uint32_t) buff[5] >> 4u) & 0x0Fu), 20);
        obj->cali_coefs.c10 = DevDPS310_GetTwoComplementOf((((uint32_t) buff[5] & 0x0Fu) << 16u) | ((uint32_t) buff[6] << 8u) | (uint32_t) buff[7], 20);
        obj->cali_coefs.c01 = DevDPS310_GetTwoComplementOf(((uint16_t) buff[8] << 8u) | (uint16_t) buff[9], 16);
        obj->cali_coefs.c11 = DevDPS310_GetTwoComplementOf(((uint16_t) buff[10] << 8u) | (uint16_t) buff[11], 16);
        obj->cali_coefs.c20 = DevDPS310_GetTwoComplementOf(((uint16_t) buff[12] << 8u) | (uint16_t) buff[13], 16);
        obj->cali_coefs.c21 = DevDPS310_GetTwoComplementOf(((uint16_t) buff[14] << 8u) | (uint16_t) buff[15], 16);
        obj->cali_coefs.c30 = DevDPS310_GetTwoComplementOf(((uint16_t) buff[16] << 8u) | (uint16_t) buff[17], 16);

        return true;
    }

    return false;
}

