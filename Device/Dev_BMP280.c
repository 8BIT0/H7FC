#include "Dev_BMP280.h"

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
        else if (obj->bus == DevBMP280_Bus_SPI)
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

static uint16_t DevBMP280_Get_ModuleID(DevBMP280Obj_TypeDef *obj)
{
    if (obj)
    {

    }

    return 0;
}

static uint16_t DevBMP280_Register_Read()
{

}

static uint16_t DevBMP280_Register_Write()
{
    
}
