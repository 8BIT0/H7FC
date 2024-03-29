#include "Dev_BMP280.h"
#include <math.h>

#define DevBMP280_Write_Mask(x) (x & ~(1 << 7))
#define DevBMP280_Read_Mask(x) (x | (1 << 7))

/* internal function */
static uint16_t DevBMP280_Register_Read(DevBMP280Obj_TypeDef *obj, uint8_t reg, uint8_t *p_buf, uint16_t len);
static uint16_t DevBMP280_Register_Write(DevBMP280Obj_TypeDef *obj, uint8_t reg, uint8_t p_buf);
static bool DevBMP280_Check_ModuleID(DevBMP280Obj_TypeDef *obj);
static bool DevBMP280_SoftReset(DevBMP280Obj_TypeDef *obj);
static bool DevBMP280_GetStatus(DevBMP280Obj_TypeDef *obj, DevBMP280_Status_TypeDef *status);
static bool DevBMP280_Calibration(DevBMP280Obj_TypeDef *obj);
static bool DevBMP280_Set_Pressure_OverSampling(DevBMP280Obj_TypeDef *obj, DevBMP280_OverSampling_List OverSampling);
static bool DevBMP280_Set_Temperature_OverSampling(DevBMP280Obj_TypeDef *obj, DevBMP280_OverSampling_List OverSampling);
static bool DevBMP280_Get_Pressure_OverSampling(DevBMP280Obj_TypeDef *obj, DevBMP280_OverSampling_List *OverSampling);
static bool DevBMP280_Get_Temperature_OverSampling(DevBMP280Obj_TypeDef *obj, DevBMP280_OverSampling_List *OverSampling);
static bool DevBMP280_Set_Filter(DevBMP280Obj_TypeDef *obj, DevBMP280_Filter_List filter);
static bool DevBMP_Get_Filter(DevBMP280Obj_TypeDef *obj, DevBMP280_Filter_List *filter);

/* external function */
static bool DevBMP280_Init(DevBMP280Obj_TypeDef *obj);
static bool DevBMP280_Sample(DevBMP280Obj_TypeDef *obj);
static bool DevBMP280_Get_DataReady(DevBMP280Obj_TypeDef *obj);
static bool DevBMP280_Get_Data(DevBMP280Obj_TypeDef *obj, float *pressure, float *temperature);

DevBMP280_TypeDef DevBMP280 = {
    .init =  DevBMP280_Init,
    .sample = DevBMP280_Sample,
    .get_data_ready = DevBMP280_Get_DataReady,
    .get_data = DevBMP280_Get_Data,
};

static bool DevBMP280_Init(DevBMP280Obj_TypeDef *obj)
{
    if (obj)
    {
        if ((obj->get_tick == NULL) || (obj->delay_ms == NULL))
        {
            obj->ErrorCode = DevBMP280_Init_Error;
            return false;
        }

        obj->ErrorCode = DevBMP280_Error_None;

        if (obj->Bus == DevBMP280_Bus_IIC)
        {
            if ((obj->send == NULL) || (obj->recv == NULL))
            {
                obj->ErrorCode = DevBMP280_Para_Error;
                return false;
            }
        }
        else if (obj->Bus == DevBMP280_Bus_SPI)
        {
            if ((obj->send == NULL) || \
                (obj->recv == NULL) || \
                (obj-> trans == NULL))
            {
                obj->ErrorCode = DevBMP280_Para_Error;
                return false;
            }

            /* check id first */
            if (!DevBMP280_Check_ModuleID(obj))
            {
                obj->ErrorCode = DevBMP280_ID_Error;
                return false;
            }

            /* soft reset */
            if (!DevBMP280_SoftReset(obj))
            {
                obj->ErrorCode = DevBMP280_Reset_Error;
                return false;
            }

            if (!DevBMP280_Calibration(obj))
            {
                obj->ErrorCode = DevBMP280_Get_CalibParam_Error;
                return false;
            }

            /* set oversimpling */
            if (!DevBMP280_Set_Pressure_OverSampling(obj, DevBMP280_OVERSAMPLING_x16))
            {
                obj->ErrorCode = DevBMP280_Set_Pressure_OverSampling_Error;
                return false;
            }

            if (!!DevBMP280_Set_Temperature_OverSampling(obj, DevBMP280_OVERSAMPLING_x2))
            {
                obj->ErrorCode = DevBMP280_Set_Temperature_OverSampling_Error;
                return false;
            }

            /* set filter */
            if (!DevBMP280_Set_Filter(obj, DevBMP280_FILTER_COEFF_16))
            {
                obj->ErrorCode = DevBMP280_Set_Filter_Error;
                return false;
            }

            return true;
        }
    }

    return false;
}

static bool DevBMP280_Set_Pressure_OverSampling(DevBMP280Obj_TypeDef *obj, DevBMP280_OverSampling_List OverSampling)
{
    uint8_t cur_setting = 0;
    uint8_t setting_readout = 0;

    if (obj)
    {
        if (obj->Bus == DevBMP280_Bus_IIC)
        {
            /* developping */
        }
        else if (obj->Bus == DevBMP280_Bus_SPI)
        {
            if (DevBMP280_Register_Read(obj, BMP280_REG_CTRL_MEAS, &cur_setting, sizeof(cur_setting)) == 0)
                return false;
        
            /* clear current setting */
            cur_setting &= ~(7 << 5);

            /* set incoming param */
            cur_setting |= (OverSampling << 5);
            
            if (DevBMP280_Register_Write(obj, BMP280_REG_CTRL_MEAS, cur_setting) == 0)
                return false;

            /* after set we need to read out the setting value and check again */
            if (DevBMP280_Register_Read(obj, BMP280_REG_CTRL_MEAS, &setting_readout, sizeof(setting_readout)) == 0)
                return false;

            if (setting_readout != cur_setting)
                return false;

            return true;
        }
    }

    return false;
}

static bool DevBMP280_Get_Pressure_OverSampling(DevBMP280Obj_TypeDef *obj, uint8_t *OverSampling)
{
    if (obj && OverSampling)
    {
        if (obj->Bus == DevBMP280_Bus_IIC)
        {
            /* developping */
        }
        else if (obj->Bus == DevBMP280_Bus_SPI)
        {
            if (DevBMP280_Register_Read(obj, BMP280_REG_CTRL_MEAS, OverSampling, sizeof(uint8_t)) == 0)
                return false;

            return true;
        }
    }

    return false;
}

static bool DevBMP280_Set_Temperature_OverSampling(DevBMP280Obj_TypeDef *obj, DevBMP280_OverSampling_List OverSampling)
{
    uint8_t cur_setting = 0;
    uint8_t setting_readout = 0;

    if (obj)
    {
        if (obj->Bus == DevBMP280_Bus_IIC)
        {
            /* developping */
        }
        else if (obj->Bus == DevBMP280_Bus_SPI)
        {
            if (DevBMP280_Register_Read(obj, BMP280_REG_CTRL_MEAS, &cur_setting, sizeof(cur_setting)) == 0)
                return false;
        
            /* clear current setting */
            cur_setting &= ~(7 << 2);

            /* set incoming param */
            cur_setting |= (OverSampling << 2);
            
            if (DevBMP280_Register_Write(obj, BMP280_REG_CTRL_MEAS, cur_setting) == 0)
                return false;

            /* after set we need to read out the setting value and check again */
            if (DevBMP280_Register_Read(obj, BMP280_REG_CTRL_MEAS, &setting_readout, sizeof(setting_readout)) == 0)
                return false;

            if (setting_readout != cur_setting)
                return false;

            return true;
        }
    }

    return false;
}

static bool DevBMP280_Get_Temperature_OverSampling(DevBMP280Obj_TypeDef *obj, uint8_t *OverSampling)
{
    if (obj && OverSampling)
    {
        if (obj->Bus == DevBMP280_Bus_IIC)
        {

        }
        else if (obj->Bus == DevBMP280_Bus_SPI)
        {
            if (DevBMP280_Register_Read(obj, BMP280_REG_CTRL_MEAS, OverSampling, sizeof(uint8_t)) == 0)
                return false;
        
            return true;
        }
    }

    return false;
}

static bool DevBMP280_Set_Filter(DevBMP280Obj_TypeDef *obj, DevBMP280_Filter_List filter)
{
    uint8_t cur_setting = 0;
    uint8_t setting_readout = 0;

    if (obj)
    {
        if (obj->Bus == DevBMP280_Bus_IIC)
        {
            /* developping */
        }
        else if (obj->Bus == DevBMP280_Bus_SPI)
        {
            if (DevBMP280_Register_Read(obj, BMP280_REG_CONFIG, &cur_setting, sizeof(cur_setting)) == 0)
                return false;

            cur_setting &= ~(7 << 2);
            cur_setting |= (filter & 0x07) << 2;

            if (DevBMP280_Register_Write(obj, BMP280_REG_CONFIG, cur_setting) == 0)
                return false;
            
            /* after set we need to read out the setting value and check again */
            if (DevBMP280_Register_Read(obj, BMP280_REG_CONFIG, &setting_readout, sizeof(setting_readout)) == 0)
                return false;

            if (setting_readout != cur_setting)
                return false;

            return true;
        }
    }

    return false;
}

static bool DevBMP_Get_Filter(DevBMP280Obj_TypeDef *obj, DevBMP280_Filter_List *filter)
{
    if (obj && filter)
    {
        if (obj->Bus == DevBMP280_Bus_IIC)
        {
            /* developping */
        }
        else if (obj->Bus == DevBMP280_Bus_SPI)
        {
            if (DevBMP280_Register_Read(obj, BMP280_REG_CONFIG, filter, sizeof(uint8_t)) == 0)
                return false;

            return true;
        }
    }

    return false;
}

static bool DevBMP280_Compensate_Temperature(DevBMP280Obj_TypeDef *obj)
{
    bool state = true;
    float var1 = 0.0f;
    float var2 = 0.0f;
    
    if (obj)
    {
        var1 = ((float)obj->raw_temperature) / 16384.0f;
        var1 -= ((float)obj->calib.t1) / 1024.0f;
        var1 *= (float)obj->calib.t2;

        var2 = ((float)obj->raw_temperature) / 131072.0f - ((float)obj->calib.t1) / 8192.0f;
        var2 *= ((float)obj->raw_temperature) / 131072.0f - ((float)obj->calib.t1) / 8192.0f;
        var2 *= ((float)obj->calib.t3);

        obj->calib.t_fine = (int32_t)(var1 + var2);
        obj->raw_temperature = (var1 + var2) / 5120.0f;
        
        if (obj->raw_temperature < -40.0f)
        {
            obj->raw_temperature = -40.0f;
            state = false;
        }
        else if (obj->raw_temperature > 85.0f)
        {
            obj->raw_temperature = 85.0f;
            state = false;
        }

        obj->temperature = obj->raw_temperature;
        return state;
    }

    return false;
}

static bool DevBMP280_Compensate_Pressure(DevBMP280Obj_TypeDef *obj)
{
    float var1 = 0.0f;
    float var2 = 0.0f;
    bool state = true;

    if (obj)
    {
        var1 = ((float)obj->calib.t_fine / 2.0f) - 64000.0f;
        var2 = pow(var1, 2) * ((float)obj->calib.p6) / 32768.0f;
        var2 = var2 + var1 * ((float)obj->calib.p5) * 2.0f;
        var2 = (var2 / 4.0f) + (((float)obj->calib.p4) * 65536.0f);
        var1 = (((float)obj->calib.p3) * pow(var1, 2) / 524288.0f + ((float)obj->calib.p2) * var1) / 524288.0f;
        var1 = (1.0f + var1 / 32768.0f) * ((float)obj->calib.p1);
        obj->raw_pressure = 0.0f;

        if ((var1 < 0.0f) || \
            (var1 > 0.0f))
        {
            obj->raw_pressure = 1048576.0f - obj->raw_pressure;
            obj->raw_pressure = (obj->raw_pressure - (var2 / 4096.0f)) * 6250.0f / var1;
            var1 = ((float)obj->calib.p9) * pow(obj->raw_pressure, 2) / 2147483648.0f;
            var2 = obj->raw_pressure * ((float)obj->calib.p8) / 32768.0f;
            obj->raw_pressure += (var1 + var2 + ((float)obj->calib.p7)) / 16.0f;
            
            if (obj->raw_pressure < 30000.0f)
            {
                obj->raw_pressure = 30000.0f;
                state = false;
            }
            else if (obj->raw_pressure > 110000.0f)
            {
                obj->raw_pressure = 110000.0f;
                state = false;
            }

            obj->pressure = obj->raw_pressure;
            return state;
        }
            
        obj->pressure = obj->raw_pressure;
    }

    return false;
}

static bool DevBMP280_GetStatus(DevBMP280Obj_TypeDef *obj, DevBMP280_Status_TypeDef *status)
{
    if (obj && status)
    {
        if (obj->Bus == DevBMP280_Bus_IIC)
        {
            /* developping */
        }
        else if (obj->Bus == DevBMP280_Bus_SPI)
        {
            if (DevBMP280_Register_Read(obj, BMP280_REG_STATUS, status->val, sizeof(uint8_t)) == 0)
                return false;

            return true;
        }
    }

    return false;
}

static bool DevBMP280_Get_DataReady(DevBMP280Obj_TypeDef *obj)
{
    DevBMP280_Status_TypeDef status;

    if (obj)
    {
        if (!DevBMP280_GetStatus(obj, &status))
            return false;

        if (status.bit.im_update && !status.bit.measuring)
            return true;
    }

    return false;
}

static bool DevBMP280_Get_Data(DevBMP280Obj_TypeDef *obj, float *pressure, float *temperature)
{
    if (obj && pressure && temperature)
    {
        *pressure = obj->pressure;
        *temperature = obj->temperature;
    }

    return false;
}

static bool DevBMP280_SoftReset(DevBMP280Obj_TypeDef *obj)
{
    uint8_t reg = DevBMP280_Write_Mask(BMP280_REG_RESET);
    uint16_t state = 0;

    if (obj)
    {
        if (obj->Bus == DevBMP280_Bus_IIC)
        {
            /* developping */
        }
        else if (obj->Bus == DevBMP280_Bus_SPI)
        {
            if (obj->delay_ms && \
                obj->trans)
            {
                if (obj->send(&reg, sizeof(reg)))
                {
                    obj->delay_ms(10);
                    return true;
                }
            }
        }
    }

    return false;
}

static bool DevBMP280_Check_ModuleID(DevBMP280Obj_TypeDef *obj)
{
    uint8_t ID = 0;

    if (obj && \
        DevBMP280_Register_Read(obj, BMP280_REG_ID, &ID, 1) && \
        (ID == BMP280_DEVICE_ID))
        return true;

    return false;
}

static bool DevBMP280_Calibration(DevBMP280Obj_TypeDef *obj)
{
    uint8_t rx_tmp[2] = {0};
    uint16_t state = 0;

    if (obj)
    {
        if (obj->Bus == DevBMP280_Bus_IIC)
        {

        }
        else if (obj->Bus == DevBMP280_Bus_SPI)
        {
            if (obj->trans)
                return false;

            /* get param t1 */
            state = DevBMP280_Register_Read(obj, BMP280_REG_NVM_PAR_T1_L, rx_tmp, sizeof(rx_tmp));
            if (state == 0)
                return false;

            obj->calib.t1 = (uint16_t)(rx_tmp[1] << 8 | rx_tmp[0]);
            memset(rx_tmp, 0, sizeof(rx_tmp));

            /* get param t2 */
            state = DevBMP280_Register_Read(obj, BMP280_REG_NVM_PAR_T2_L, rx_tmp, sizeof(rx_tmp));
            if (state == 0)
                return false;

            obj->calib.t2 = (uint16_t)(rx_tmp[1] << 8 | rx_tmp[0]);
            memset(rx_tmp, 0, sizeof(rx_tmp));

            /* get param t3 */
            state = DevBMP280_Register_Read(obj, BMP280_REG_NVM_PAR_T3_L, rx_tmp, sizeof(rx_tmp));
            if (state == 0)
                return false;

            obj->calib.t3 = (uint16_t)(rx_tmp[1] << 8 | rx_tmp[0]);
            memset(rx_tmp, 0, sizeof(rx_tmp));

            /* get param p1 */
            state = DevBMP280_Register_Read(obj, BMP280_REG_NVM_PAR_P1_L, rx_tmp, sizeof(rx_tmp));
            if (state == 0)
                return false;

            obj->calib.p1 = (uint16_t)(rx_tmp[1] << 8 | rx_tmp[0]);
            memset(rx_tmp, 0, sizeof(rx_tmp));
            
            /* get param p2 */
            state = DevBMP280_Register_Read(obj, BMP280_REG_NVM_PAR_P2_L, rx_tmp, sizeof(rx_tmp));
            if (state == 0)
                return false;

            obj->calib.p2 = (uint16_t)(rx_tmp[1] << 8 | rx_tmp[0]);
            memset(rx_tmp, 0, sizeof(rx_tmp));

            /* get param p3 */
            state = DevBMP280_Register_Read(obj, BMP280_REG_NVM_PAR_P3_L, rx_tmp, sizeof(rx_tmp));
            if (state == 0)
                return false;

            obj->calib.p3 = (uint16_t)(rx_tmp[1] << 8 | rx_tmp[0]);
            memset(rx_tmp, 0, sizeof(rx_tmp));

            /* get param p4 */
            state = DevBMP280_Register_Read(obj, BMP280_REG_NVM_PAR_P4_L, rx_tmp, sizeof(rx_tmp));
            if (state == 0)
                return false;

            obj->calib.p4 = (uint16_t)(rx_tmp[1] << 8 | rx_tmp[0]);
            memset(rx_tmp, 0, sizeof(rx_tmp));

           /* get param p5 */
            state = DevBMP280_Register_Read(obj, BMP280_REG_NVM_PAR_P5_L, rx_tmp, sizeof(rx_tmp));
            if (state == 0)
                return false;

            obj->calib.p5 = (uint16_t)(rx_tmp[1] << 8 | rx_tmp[0]);
            memset(rx_tmp, 0, sizeof(rx_tmp));

           /* get param p6 */
            state = DevBMP280_Register_Read(obj, BMP280_REG_NVM_PAR_P6_L, rx_tmp, sizeof(rx_tmp));
            if (state == 0)
                return false;

            obj->calib.p6 = (uint16_t)(rx_tmp[1] << 8 | rx_tmp[0]);
            memset(rx_tmp, 0, sizeof(rx_tmp));

           /* get param p7 */
            state = DevBMP280_Register_Read(obj, BMP280_REG_NVM_PAR_P7_L, rx_tmp, sizeof(rx_tmp));
            if (state == 0)
                return false;

            obj->calib.p7 = (uint16_t)(rx_tmp[1] << 8 | rx_tmp[0]);
            memset(rx_tmp, 0, sizeof(rx_tmp));

           /* get param p8 */
            state = DevBMP280_Register_Read(obj, BMP280_REG_NVM_PAR_P8_L, rx_tmp, sizeof(rx_tmp));
            if (state == 0)
                return false;

            obj->calib.p8 = (uint16_t)(rx_tmp[1] << 8 | rx_tmp[0]);
            memset(rx_tmp, 0, sizeof(rx_tmp));

           /* get param p9 */
            state = DevBMP280_Register_Read(obj, BMP280_REG_NVM_PAR_P9_L, rx_tmp, sizeof(rx_tmp));
            if (state == 0)
                return false;

            obj->calib.p9 = (uint16_t)(rx_tmp[1] << 8 | rx_tmp[0]);
            memset(rx_tmp, 0, sizeof(rx_tmp));

            obj->calib.t_fine = 0;

            return true;
        }
    }

    return false;
}

static bool DevBMP280_Sample(DevBMP280Obj_TypeDef *obj)
{
    uint8_t mode = 0;
    uint32_t timeout = 0;
    uint8_t buf[6] = {0};
    
    if (obj && obj->delay_ms)
    {
        if (obj->Bus == DevBMP280_Bus_IIC)
        {
            /* developping */
        }
        else if (obj->Bus == DevBMP280_Bus_SPI)
        {
            if (DevBMP280_Register_Read(obj, BMP280_REG_CTRL_MEAS, &mode, 1) == 0)
                return false;
            
            if ((mode & 0x3) == BMP280_NORMAL_MODE)
            {
                if(DevBMP280_Register_Read(obj, BMP280_REG_PRESS_MSB, buf, sizeof(buf)) == 0)
                    return false;

                obj->raw_temperature = ((((uint32_t)(buf[3])) << 12) | (((uint32_t)(buf[4])) << 4) | ((uint32_t)buf[5] >> 4));
                if (!DevBMP280_Compensate_Temperature(obj))
                    return false;

                obj->raw_pressure = ((((int32_t)(buf[0])) << 12) | (((int32_t)(buf[1])) << 4) |(((int32_t)(buf[2])) >> 4));
                if (!DevBMP280_Compensate_Pressure(obj))
                    return false;
            }
            else
            {
                /* forced mode */
                if (DevBMP280_Register_Read(obj, BMP280_REG_CTRL_MEAS, &mode, 1) == 0)
                    return false;
                
                mode &= ~(3 << 0);
                mode |= 0x01 << 0;

                if (DevBMP280_Register_Write(obj, BMP280_REG_CTRL_MEAS, mode) == 0)
                    return false;

                timeout = 10 * 1000;
                while (timeout != 0)
                {
                    if (DevBMP280_Register_Read(obj, BMP280_REG_CTRL_MEAS, &mode, 1) == 0)
                        return false;
                    
                    if ((mode & 0x03) == 0)
                        break;
                    
                    obj->delay_ms(1);
                    timeout--;

                    if (timeout == 0)
                        return false;
                }

                if (DevBMP280_Register_Read(obj, BMP280_REG_PRESS_MSB, buf, 6) == 0)
                    return false;

                obj->raw_temperature = ((((uint32_t)(buf[3])) << 12) | (((uint32_t)(buf[4])) << 4) | ((uint32_t)buf[5] >> 4));
                if (!DevBMP280_Compensate_Temperature(obj))
                    return false;
                
                obj->raw_pressure = ((((int32_t)(buf[0])) << 12) | (((int32_t)(buf[1])) << 4) | (((int32_t)(buf[2])) >> 4));
                if (!DevBMP280_Compensate_Pressure(obj))
                    return false;
            }

            return true;
        }
    }

    return false;
}

static uint16_t DevBMP280_Register_Read(DevBMP280Obj_TypeDef *obj, uint8_t reg, uint8_t *p_buf, uint16_t len)
{
    uint8_t tx_tmp[64] = {0};
    uint8_t rx_tmp[64] = {0};
    uint16_t state = 0;

    if (obj && p_buf && (len < 64))
    {
        if (obj->Bus == DevBMP280_Bus_IIC)
        {
            /* developping */
        }
        else if (obj->Bus == DevBMP280_Bus_SPI)
        {
            if (obj->trans)
            {
                tx_tmp[0] = DevBMP280_Read_Mask(reg);
                state = obj->trans(tx_tmp, rx_tmp, len);

                return state;
            }
        }
    }

    return 0;
}

static uint16_t DevBMP280_Register_Write(DevBMP280Obj_TypeDef *obj, uint8_t reg, uint8_t p_buf)
{
    uint8_t tx_tmp[2] = {0};
    uint8_t rx_tmp[2] = {0};
    uint16_t state = 0;
    
    if (obj)
    {
        if (obj->Bus == DevBMP280_Bus_IIC)
        {
            /* developping */
        }
        else if (obj->Bus == DevBMP280_Bus_SPI)
        {
            if (obj->trans)
            {
                tx_tmp[0] = DevBMP280_Write_Mask(reg);
                tx_tmp[1] = p_buf;

                state = obj->trans(tx_tmp, rx_tmp, sizeof(tx_tmp));
                return state;
            }
        }
    }

    return 0;
}

// static uint16_t DevBMP280_Register_Trans(DevBMP280Obj_TypeDef *obj, uint8_t reg, uint8_t *tx, uint8_t *rx, uint16_t len)
// {
//     uint8_t tx_tmp[2] = {0};
//     uint8_t rx_tmp[2] = {0};
//     uint16_t state = 0;
    
//     if (obj)
//     {
//         if (obj->Bus == DevBMP280_Bus_IIC)
//         {
//             /* developping */
//         }
//         else if (obj->Bus == DevBMP280_Bus_SPI)
//         {
//             if (obj->cs_ctl && obj->trans)
//             {
//                 tx_tmp[0] = DevBMP280_Write_Mask(reg);
//                 tx_tmp[1] = p_buf;

//                 obj->cs_ctl(false);
//                 state = obj->trans(tx_tmp, rx_tmp, sizeof(tx_tmp));
//                 obj->cs_ctl(true);

//                 return state;
//             }
//         }
//     }

//     return 0;
// }
