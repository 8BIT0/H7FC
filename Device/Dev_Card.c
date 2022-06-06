#include "Dev_Card.h"
#include "IO_Definition.h"

/* External Function */
static bool DevCard_Init(DevCard_Obj_TypeDef Instance);

DevCard_TypeDef DevCard = {
    .Init = DevCard_Init,
};

static bool DevCard_Init(DevCard_Obj_TypeDef Instance)
{
    BspSDMMC.init(&(Instance.SDMMC_Obj));
}
