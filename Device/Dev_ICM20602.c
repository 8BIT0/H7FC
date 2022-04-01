#include "Dev_ICM20602.h"

static void DevICM20602_SetSampleRate(DevICM20602Obj_TypeDef *Obj, ICM20602_SampleRate_List rate)
{
    Obj->rate = rate;
}

static bool DevICM20602_PreInit()
{
}

static ICM20602_Error_List DevICM20602_Init(DevICM20602Obj_TypeDef *Obj)
{
    return ICM20602_No_Error;
}

static void DevICM20602_SetDRDY(DevICM20602Obj_TypeDef *Obj)
{
    Obj->drdy = true;
}

static bool DevICM20602_SwReset()
{
}

static bool DevICM20602_GetReady(DevICM20602Obj_TypeDef *Obj)
{
    return Obj->drdy;
}

static void DevICM20602_Sample()
{
}

static IMUData_TypeDef DevICM20602_Get_Data()
{
}

static ICM20602_Error_List DevICM20602_Get_InitError()
{
}
