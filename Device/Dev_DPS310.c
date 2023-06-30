#include "Dev_DPS310.h"

#define DPS310_SAMPLE_RATE DPS310_CFG_RATE_32_MEAS
#define DPS310_PROC_TIME   DPS310_PRS_CFG_PM_PRC_16_TIMES
#define DPS310_MAX_SAMPLE_RATE 10 /* unit:ms period 10ms 100Hz */
#define DPS310_MIN_SAMPLE_RATE 25 /* unit:ms period 25ms 40Hz  */

/* internal function */
static bool DevDPS310_WriteByteToReg(DevDPS310Obj_TypeDef *obj, uint8_t reg_addr, uint8_t data);
static int16_t DevDPS310_Temperature_Sensor(DevDPS310Obj_TypeDef *obj, uint8_t *p_sensor);
static bool DevDPS310_Get_ProdID(DevDPS310Obj_TypeDef *obj);
static uint32_t DevDPS310_GetScale(DevDPS310Obj_TypeDef *obj);
static bool DevDPS310_Sleep(DevDPS310Obj_TypeDef *obj);
static bool DevDPS310_Reset(DevDPS310Obj_TypeDef *obj);
static int32_t DevDPS310_GetTwoComplementOf(uint32_t value, uint8_t length);

/* external function */
static bool DevDPS310_PreInit(DevDPS310Obj_TypeDef *obj, DevDPS310_BusWrite write, DevDPS310_BusRead read);
static bool DevDPS310_Init(DevDPS310Obj_TypeDef *obj);

/* external object */
DevDPS310_TypeDef DevDPS310 = {
    .per_init = DevDPS310_PreInit,
    .init = DevDPS310_Init,
};

static bool DevDPS310_PreInit(DevDPS310Obj_TypeDef *obj, DevDPS310_BusWrite write, DevDPS310_BusRead read)
{
    if(obj && write && read)
    {
        obj->bus_tx = write;
        obj->bus_rx = read;

        return true;
    }

    return false;
}

static bool DevDPS310_Init(DevDPS310Obj_TypeDef *obj)
{
    if(obj  && obj->bus_delay)
    {
        if(!DevDPS310_Get_ProdID(obj))
        {
            obj->error = DevDPS310_Error_BadID;
            return false;
        }

        DevDPS310_Reset(obj);
        obj->bus_delay(50);

        /* configure pressure sensor */
        DevDPS310_WriteByteToReg(obj, DPS310_PRS_CFG_REG, DPS310_SAMPLE_RATE | DPS310_PROC_TIME);
        obj->pres_factory_scale = DevDPS310_GetScale(obj, DPS310_SAMPLE_RATE);

        /* configure temprature sensor */
        obj->temp_factory_scale = DevDPS310_GetScale(obj, DPS310_SAMPLE_RATE);

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
                obj->factory_scale = 524288;
                return 524288;
                
            case DPS310_CFG_RATE_2_MEAS:
                obj->factory_scale = 1572864;
                return 1572864;

            case DPS310_CFG_RATE_4_MEAS:
                obj->factory_scale = 3670016;
                return 3670016;

            case DPS310_CFG_RATE_8_MEAS:
                obj->factory_scale = 7864320;
                return 7864320;

            case DPS310_CFG_RATE_16_MEAS:
                obj->factory_scale = 253952;
                return 253952;

            case DPS310_CFG_RATE_32_MEAS:
                obj->factory_scale = 516096;
                return 516096;

            case DPS310_CFG_RATE_64_MEAS:
                obj->factory_scale = 1040384;
                return 1040384;

            case DPS310_CFG_RATE_128_MEAS:
                obj->factory_scale = 2088960;
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
        if(DevDPS310_Temperature_Sensor(&temperature_sensor))
        {
            g_temperature_rate = DPS310_TMP_CFG_TMP_RATE_MASK & `data;
            data |= temperature_sensor;
        }

        return DevDPS310_WriteByteToReg(DPS310_TMP_CFG_REG, data);
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

