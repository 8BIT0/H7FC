#include "DiskIO.h"
#include "Dev_W25Qxx.h"
#include "Dev_Card.h"
#include "IO_Definition.h"
#include "error_log.h"

static Disk_Info_TypeDef Disk_Info;

#if (STORAGE_MODULE & EXTERNAL_INTERFACE_TYPE_SPI_FLASH)
DevW25QxxObj_TypeDef W25Q64_Obj = {
    .bus_type = DevW25Qxx_Norm_SpiBus,
    .BusPort = SPI1,
};
#endif

#if (STORAGE_MODULE & EXTERNAL_INTERFACE_TYPE_TF_CARD)
static const BspSDMMC_PinConfig_TypeDef SDMMC_Pin = {
    .CK_Port = SDMMC_CLK_PORT,
    .CMD_Port = SDMMC_CMD_PORT,
    .D0_Port = D0_PORT,
    .D1_Port = D1_PORT,
    .D2_Port = D2_PORT,
    .D3_Port = D3_PORT,

    .CK_Pin = SDMMC_CLK_PIN,
    .CMD_Pin = SDMMC_CMD_PIN,
    .D0_Pin = D0_PIN,
    .D1_Pin = D1_PIN,
    .D2_Pin = D2_PIN,
    .D3_Pin = D3_PIN,

    .Alternate = GPIO_AF12_SDIO1,
};

static BspSDMMC_Obj_TypeDef SDMMC_Instance = {
    .instance = SDMMC1,
};

static DevCard_Obj_TypeDef DevTFCard_Obj = {
    .SDMMC_Obj = SDMMC_Instance,
};
#endif

bool ExtDisk_Init(void)
{
    bool init_state = false;

#if (STORAGE_MODULE & EXTERNAL_INTERFACE_TYPE_SPI_FLASH)
    init_state &= DevW25Q64.init(W25Q64_Obj);
#endif

#if (STORAGE_MODULE & EXTERNAL_INTERFACE_TYPE_TF_CARD)
    init_state &= DevCard.Init(DevTFCard_Obj);
#endif

    return init_state;
}

#if (STORAGE_MODULE & INTERNAL_INTERFACE_TYPE)
bool IntDisk_Init(void)
{
    bool init_state = false;

    return init_state;
}
#endif

bool Disk_Init(void)
{
    memset(&Disk_Info, NULL, sizeof(Disk_Info));

#if (STORAGE_MODULE & INTERNAL_INTERFACE_TYPE)
    if (!IntDisk_Init())
        return false;
#endif

#if ((STORAGE_MODULE & EXTERNAL_INTERFACE_TYPE_TF_CARD) || (STORAGE_MODULE & EXTERNAL_INTERFACE_TYPE_SPI_FLASH))
    if (ExtDisk_Init() != STORAGE_MODULE_NO_ERROR)
        return false;
#endif

    return true;
}
