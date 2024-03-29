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
static bool DevCMP280_Calibration(DevBMP280Obj_TypeDef *obj);

/* external function */
static bool DevBMP280_Init(DevBMP280Obj_TypeDef *obj);

DevBMP280_TypeDef DevBMP280 = {
    .init =  DevBMP280_Init,
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
            if ((obj->cs_ctl == NULL) || \
                (obj->send == NULL) || \
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

            if (!DevCMP280_Calibration(obj))
            {
                obj->ErrorCode = DevBMP280_Get_CalibParam_Error;
                return false;
            }

            return true;
        }
    }

    return false;
}

static bool DevBMP280_Set_Pressure_OverSampling(DevBMP280Obj_TypeDef *obj, DevBMP280_OverSampling_List OverSampling)
{
    if (obj)
    {
        if (obj->Bus == DevBMP280_Bus_IIC)
        {

        }
        else if (obj->Bus == DevBMP280_Bus_SPI)
        {
            if ((obj->cs_ctl) && \
                (obj->trans))
            {

            }
        }
    }

    return false;
}

static bool DevBMP280_Set_Temperature_OverSampling(DevBMP280Obj_TypeDef *obj, DevBMP280_OverSampling_List OverSampling)
{
    if (obj)
    {
        if (obj->Bus == DevBMP280_Bus_IIC)
        {

        }
        else if (obj->Bus == DevBMP280_Bus_SPI)
        {
            if ((obj->cs_ctl) && \
                (obj->trans))
            {

            }
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
    uint16_t state = 0;
    uint16_t tx_tmp[2] = {0};
    uint16_t rx_tmp[2] = {0};

    if (obj)
    {
        if (obj->Bus == DevBMP280_Bus_IIC)
        {
            /* developping */
        }
        else if (obj->Bus == DevBMP280_Bus_SPI)
        {
            if ((obj->cs_ctl) && \
                (obj->trans))
            {
                obj->cs_ctl(false);
                state = obj->trans(tx_tmp, rx_tmp, sizeof(tx_tmp));
                obj->cs_ctl(true);
            
                status->val = 0;
                if (state)
                {
                    status->val = rx_tmp[1];
                    return true;
                }
            }
        }
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
                obj->cs_ctl && \
                obj->trans)
            {
                obj->cs_ctl(false);
                state = obj->send(&reg, sizeof(reg));
                obj->cs_ctl(true);

                if (state)
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

static bool DevCMP280_Calibration(DevBMP280Obj_TypeDef *obj)
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
            if ((obj->cs_ctl) || \
                (obj->trans))
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
            if (obj->cs_ctl && obj->trans)
            {
                tx_tmp[0] = DevBMP280_Read_Mask(reg);

                obj->cs_ctl(false);
                state = obj->trans(tx_tmp, rx_tmp, len);
                obj->cs_ctl(true);

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
            if (obj->cs_ctl && obj->trans)
            {
                tx_tmp[0] = DevBMP280_Write_Mask(reg);
                tx_tmp[1] = p_buf;

                obj->cs_ctl(false);
                state = obj->trans(tx_tmp, rx_tmp, sizeof(tx_tmp));
                obj->cs_ctl(true);

                return state;
            }
        }
    }

    return 0;
}
