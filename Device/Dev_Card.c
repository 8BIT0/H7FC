#include "Dev_Card.h"
#include "IO_Definition.h"

/* External Function */
static DevCard_Error_List DevCard_Init(DevCard_Obj_TypeDef *Instance);
static DevCard_Error_List DevCard_GetError(DevCard_Obj_TypeDef *Instance);

DevCard_TypeDef DevCard = {
    .Init = DevCard_Init,
    .Get_ErrorCode = DevCard_GetError,
};

static DevCard_Error_List DevCard_Init(DevCard_Obj_TypeDef *Instance)
{
    if (Instance == NULL)
        return DevCard_Obj_Error;

    if (!BspSDMMC.init(&(Instance->SDMMC_Obj)))
    {
        Instance->error_code = DevCard_Bus_Error;
        return DevCard_Bus_Error;
    }

    if (!BspSDMMC.info(&(Instance->SDMMC_Obj), &(Instance->SDMMC_Obj.info)))
    {
        Instance->error_code = DevCard_Info_Error;
        return DevCard_Info_Error;
    }

    Instance->error_code = DevCard_No_Error;
    return DevCard_No_Error;
}

static DevCard_Error_List DevCard_GetError(DevCard_Obj_TypeDef *Instance)
{
    if (Instance == NULL)
        return DevCard_Obj_Error;

    return Instance->error_code;
}

static DevCard_Error_List DevCard_GetInfo(DevCard_Obj_TypeDef *Instance)
{
    if (Instance == NULL)
        return DevCard_Obj_Error;

    return DevCard_No_Error;
}
