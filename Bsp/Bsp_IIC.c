#include "Bsp_IIC.h"

static bool BspIIC_Init(BspIICObj_TypeDef *obj)
{
    if(obj)
    {
        switch((uint8_t)(obj->instance_id))
        {
            case BspIIC_Instance_I2C_2:
                obj->handle.Instance = I2C2;
                break;

            default:
                return false;
        }
    }

    return false;
}