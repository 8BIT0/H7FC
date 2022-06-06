#include "Dev_Card.h"
#include "IO_Definition.h"

/* External Function */
static DevCard_Error_List DevCard_Init(DevCard_Obj_TypeDef Instance);

DevCard_TypeDef DevCard = {
    .Init = DevCard_Init,
};

static DevCard_Error_List DevCard_Init(DevCard_Obj_TypeDef Instance)
{
    if (!BspSDMMC.init(&(Instance.SDMMC_Obj)))
        return DevCard_Bus_Error;

    if (!BspSDMMC.info(&(Instance.SDMMC_Obj), &(Instance.SDMMC_Obj.info)))
        return DevCard_Info_Error;

    return DevCard_No_Error;
}
