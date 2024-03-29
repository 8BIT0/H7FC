#include "Dev_BMP280.h"

#define DevBMP280_Write_Mask(x) (x & ~(1 << 7))
#define DevBMP280_Read_Mask(x) (x | (1 << 7))

/* internal function */
static uint16_t DevBMP280_Register_Read(DevBMP280Obj_TypeDef *obj, uint8_t reg, uint8_t *p_buf);
static uint16_t DevBMP280_Register_Write(DevBMP280Obj_TypeDef *obj, uint8_t reg, uint8_t p_buf);

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


        }
    }

    return false;
}

static bool DevBMP280_Check_ModuleID(DevBMP280Obj_TypeDef *obj)
{
    uint8_t ID = 0;

    if (obj)
    {
        if (DevBMP280_Register_Read(obj, BMP280_REG_ID, &ID) && \
            (ID == BMP280_DEVICE_ID))
            return true;
    }

    return false;
}

static uint16_t DevBMP280_Register_Read(DevBMP280Obj_TypeDef *obj, uint8_t reg, uint8_t *p_buf)
{
    uint8_t tx_tmp[2] = {0};
    uint8_t rx_tmp[2] = {0};
    uint16_t state = 0;

    if (obj && p_buf)
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
                state = obj->trans(tx_tmp, rx_tmp, sizeof(tx_tmp));
                obj->cs_ctl(true);

                *p_buf = 0;
                if (state)
                    *p_buf = rx_tmp[1];

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
