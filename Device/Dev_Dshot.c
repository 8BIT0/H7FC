#include "Dev_Dshot.h"

static bool DevDshot_Init(DevDshotObj_TypeDef *obj, DevDshotType_List type, BspGPIO_Obj_TypeDef pin)
{
    if(!obj || (type < DevDshot_150) || (type > DevDshot_600))
        return false;

    obj->type = type;
    obj->pin = pin;

    return true;
}

