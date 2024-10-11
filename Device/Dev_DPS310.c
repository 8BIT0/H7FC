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
static float DevDPS310_GetScale(DevDPS310Obj_TypeDef *obj, uint8_t rate);
static bool DevDPS310_Reset(DevDPS310Obj_TypeDef *obj);
static bool DevDPS310_Configure_Temperature(DevDPS310Obj_TypeDef *obj, uint8_t data);
static int32_t DevDPS310_GetTwoComplementOf(uint32_t value, uint8_t length);
static bool DevDPS310_Get_Cali_Coefs(DevDPS310Obj_TypeDef *obj);
static bool DevDPS310_ReadLenByteToReg(DevDPS310Obj_TypeDef *obj, uint8_t reg_addr, uint8_t *data, uint8_t len);

/* external function */
static bool DevDPS310_Init(DevDPS310Obj_TypeDef *obj);
static bool DevDPS310_Sample(DevDPS310Obj_TypeDef *obj);
static bool DevDPS310_GetReady(DevDPS310Obj_TypeDef *obj);
static DevDPS310_Data_TypeDef DevDPS310_Get_Data(DevDPS310Obj_TypeDef *obj);

/* external object */
DevDPS310_TypeDef DevDPS310 = {
    .init = DevDPS310_Init,
    .sample = DevDPS310_Sample,
    .ready = DevDPS310_GetReady,
    .get_data = DevDPS310_Get_Data,
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

        if(!DevDPS310_Reset(obj))
        {
            obj->error = DevDPS310_Error_Reset_Filed;
            return false;
        }

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
            obj->pres_factory_scale = DevDPS310_GetScale(obj, DPS310_PROC_TIME);
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
            obj->temp_factory_scale = DevDPS310_GetScale(obj, DPS310_PROC_TIME);
        }
        else
        {
            /* temprature init error */
            obj->error = DevDPS310_Error_TempratureInit;
            return false;
        }

        // set pressure and temperature result bit-shift (required when the oversampling rate is >8 times)
        DevDPS310_WriteByteToReg(obj, DPS310_CFG_REG_REG, DPS310_CFG_RET_TMP_SHIFT_EN | DPS310_CFG_RET_PRS_SHIFT_EN);

        // Continuous pressure and temperature measurement
        DevDPS310_WriteByteToReg(obj, DPS310_MEAS_CFG_REG, DPS310_MEAS_CFG_MEAS_CTRL_CONTINUOUS_PRS_TMP);

        obj->DevAddr = DPS310_I2C_ADDR;

        return true;
    }

    return false;
}

static float DevDPS310_GetScale(DevDPS310Obj_TypeDef *obj, uint8_t rate)
{
    if(obj)
    {
        switch (rate)
        {
            case DPS310_TMP_CFG_TMP_PRC_SINGLE:
                obj->factory_scale = 524288.0f;
                return 524288.0f;
                
            case DPS310_TMP_CFG_TMP_PRC_2_TIMES:
                obj->factory_scale = 1572864.0f;
                return 1572864.0f;

            case DPS310_TMP_CFG_TMP_PRC_4_TIMES:
                obj->factory_scale = 3670016.0f;
                return 3670016.0f;

            case DPS310_TMP_CFG_TMP_PRC_8_TIMES:
                obj->factory_scale = 7864320.0f;
                return 7864320.0f;

            case DPS310_TMP_CFG_TMP_PRC_16_TIMES:
                obj->factory_scale = 253952.0f;
                return 253952.0f;

            case DPS310_TMP_CFG_TMP_PRC_32_TIMES:
                obj->factory_scale = 516096.0f;
                return 516096.0f;

            case DPS310_TMP_CFG_TMP_PRC_64_TIMES:
                obj->factory_scale = 1040384.0f;
                return 1040384.0f;

            case DPS310_TMP_CFG_TMP_PRC_128_TIMES:
                obj->factory_scale = 2088960.0f;
                return 2088960.0f;

            default:
                return 0.0f;
        }
    }

    return 0.0f;
}

static bool DevDPS310_Reset(DevDPS310Obj_TypeDef *obj)
{
    if(obj)
        return DevDPS310_WriteByteToReg(obj, DPS310_RESET_REG, DPS310_RESET_SOFT_RST_VALUE);

    return false;
}

static bool DevDPS310_Get_ProdID(DevDPS310Obj_TypeDef *obj)
{
    uint8_t data_tmp = 0;

    if(obj && obj->bus_rx && obj->bus_rx(obj->DevAddr, DPS310_PRODUCT_ID_REG, &data_tmp, 1))
    {
        obj->ProdID = data_tmp;
        if(data_tmp == DPS310_PRODUCT_ID_VALUE)
            return true;
    }

    return false;
}

static bool DevDPS310_Configure_Temperature(DevDPS310Obj_TypeDef *obj, uint8_t data)
{
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

        return true;
    }

    return false;
}

static bool DevDPS310_ReadLenByteToReg(DevDPS310Obj_TypeDef *obj, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    if(obj && obj->bus_rx && data && len)
    {
        if(!obj->bus_rx(obj->DevAddr, reg_addr, data, len))
        {
            memset(data, 0, len);
            return false;
        }

        return true;
    }

    return false;
}

static int32_t DevDPS310_GetTwoComplementOf(uint32_t value, uint8_t length)
{
    if (value & ((int)1 << (length - 1)))
    {
        return ((int32_t)value) - ((int32_t)1 << length);
    }
        
    return value;
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
        obj->cali_coefs.c0  = DevDPS310_GetTwoComplementOf(((uint32_t) buff[0] << 4u) | (((uint32_t) buff[1] >> 4u) & 0x0Fu), 12);
        obj->cali_coefs.c1  = DevDPS310_GetTwoComplementOf(((((uint32_t) buff[1] & 0x0Fu) << 8u) | (uint32_t) buff[2]), 12);
        obj->cali_coefs.c00 = DevDPS310_GetTwoComplementOf(((uint32_t) buff[3] << 12u) | ((uint32_t) buff[4] << 4u) | (((uint32_t) buff[5] >> 4u) & 0x0Fu), 20);
        obj->cali_coefs.c10 = DevDPS310_GetTwoComplementOf((((uint32_t) buff[5] & 0x0Fu) << 16u) | ((uint32_t) buff[6] << 8u) | (uint32_t) buff[7], 20);
        obj->cali_coefs.c01 = DevDPS310_GetTwoComplementOf(((uint32_t) buff[8] << 8u) | (uint32_t) buff[9], 16);
        obj->cali_coefs.c11 = DevDPS310_GetTwoComplementOf(((uint32_t) buff[10] << 8u) | (uint32_t) buff[11], 16);
        obj->cali_coefs.c20 = DevDPS310_GetTwoComplementOf(((uint32_t) buff[12] << 8u) | (uint32_t) buff[13], 16);
        obj->cali_coefs.c21 = DevDPS310_GetTwoComplementOf(((uint32_t) buff[14] << 8u) | (uint32_t) buff[15], 16);
        obj->cali_coefs.c30 = DevDPS310_GetTwoComplementOf(((uint32_t) buff[16] << 8u) | (uint32_t) buff[17], 16);

        return true;
    }

    return false;
}

/* single trigger one shot mode */
static bool DevDPS310_Sample(DevDPS310Obj_TypeDef *obj)
{
    uint8_t bus_read[6] = {0};

    if(obj)
    {
        obj->ready = false;

        // read pressure data from register
        if(!DevDPS310_ReadLenByteToReg(obj, DPS310_PSR_B2_REG, bus_read, sizeof(bus_read)))
        {
            obj->pressure = 0.0f;
            obj->tempra = 0.0f;
            return false;
        }

        obj->none_scale_pressure = 0;
        memcpy(&obj->none_scale_pressure, bus_read, 3);

        obj->none_scale_tempra = 0;
        memcpy(&obj->none_scale_tempra, &bus_read[3], 3);

        // Calculate scaled measurement results.
        const float Praw_sc = DevDPS310_GetTwoComplementOf((bus_read[0] << 16) + (bus_read[1] << 8) + bus_read[2], 24) / obj->pres_factory_scale;
        const float Traw_sc = DevDPS310_GetTwoComplementOf((bus_read[3] << 16) + (bus_read[4] << 8) + bus_read[5], 24) / obj->temp_factory_scale;

        // Calculate compensated measurement results.
        const float c00 = obj->cali_coefs.c00;
        const float c01 = obj->cali_coefs.c01;
        const float c10 = obj->cali_coefs.c10;
        const float c11 = obj->cali_coefs.c11;
        const float c20 = obj->cali_coefs.c20;
        const float c21 = obj->cali_coefs.c21;
        const float c30 = obj->cali_coefs.c30;

        // See section 4.9.1, How to Calculate Compensated Pressure Values, of datasheet
        obj->pressure = c00 + Praw_sc * (c10 + Praw_sc * (c20 + Praw_sc * c30)) + Traw_sc * c01 + Traw_sc * Praw_sc * (c11 + Praw_sc * c21);

        const float c0 = obj->cali_coefs.c0;
        const float c1 = obj->cali_coefs.c1;

        // See section 4.9.2, How to Calculate Compensated Temperature Values, of datasheet
        obj->tempra = c0 * 0.5f + c1 * Traw_sc;

        obj->cyc ++;
        if(obj->get_tick)
            obj->update_time = obj->get_tick();

        obj->ready = true;

        return true;
    }

    return false;
}

static bool DevDPS310_GetReady(DevDPS310Obj_TypeDef *obj)
{
    if(obj)
        return obj->ready;

    return false;
}

static DevDPS310_Data_TypeDef DevDPS310_Get_Data(DevDPS310Obj_TypeDef *obj)
{
    DevDPS310_Data_TypeDef tmp;

    memset(&tmp, 0, sizeof(DevDPS310_Data_TypeDef));

    if(obj)
    {
        tmp.time_stamp = obj->update_time;
        tmp.cyc = obj->cyc;
        tmp.scaled_press = obj->pressure;
        tmp.scaled_tempra = obj->tempra;

        if(obj->ready)
            obj->ready = false;
    }

    return tmp;
}
