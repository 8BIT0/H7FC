#include "DiskIO.h"
#include "Dev_W25Qxx.h"
#include "Dev_Card.h"
#include "IO_Definition.h"
#include "error_log.h"

static Disk_Info_TypeDef Disk_Info;

static Error_Obj_Typedef DevCard_ErrorList[] = {
    {
        .out = true,
        .log = false,
        .prc_callback = NULL,
        .code = DevCard_Internal_Module_Init_Error,
        .desc = "internal Storage Init Error\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = NULL,
        .code = DevCard_External_Module_Init_Error,
        .desc = "External Storage Init Error\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
}

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

static void DevCard_ExtModule_InitError(int16_t code, uint8_t *p_arg, uint16_t size)
{
}
