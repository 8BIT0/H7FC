#include "DiskIO.h"
#include "Dev_W25Qxx.h"
#include "Dev_Card.h"
#include "IO_Definition.h"
#include "error_log.h"
#include <stdarg.h>
#include <stdio.h>

static uint8_t Disk_Print_Buff[128] = {0};
static Disk_Printf_Callback Disk_PrintOut = NULL;

static Error_Handler DevCard_Error_Handle = NULL;
static Disk_Info_TypeDef Disk_Info;

/* Internal Function */
static void Disk_ExtModule_InitError(int16_t code, uint8_t *p_arg, uint16_t size);
static void Disk_Printf(const char *str, ...);

#if (STORAGE_MODULE & EXTERNAL_INTERFACE_TYPE_SPI_FLASH)
/******************************************************************************** SPI Interface **************************************************************************/
DevW25QxxObj_TypeDef W25Q64_Obj = {
    .bus_type = DevW25Qxx_Norm_SpiBus,
    .BusPort = SPI1,
};
#endif

#if (STORAGE_MODULE & EXTERNAL_INTERFACE_TYPE_TF_CARD)
/******************************************************************************** SDMMC Interface **************************************************************************/
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

static DevCard_Obj_TypeDef DevTFCard_Obj = {
    .SDMMC_Obj = {
        .instance = SDMMC1,
    },
};
#endif

/******************************************************************************* Error Proc Object **************************************************************************/
static Error_Obj_Typedef DevCard_ErrorList[] = {
#if (STORAGE_MODULE & INTERNAL_INTERFACE_TYPE)
    {
        .out = true,
        .log = false,
        .prc_callback = NULL,
        .code = DevCard_Internal_Module_Init_Error,
        .desc = "internal Storage Init Error\r\n",
        .proc_type = Error_Proc_Immd,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
#endif
    {
        .out = true,
        .log = false,
        .prc_callback = Disk_ExtModule_InitError,
        .code = DevCard_External_Module_Init_Error,
        .desc = "External Storage Init Error\r\n",
        .proc_type = Error_Proc_Immd,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
};

static bool ExtDisk_Init(void)
{
    bool init_state = true;

    memset(&Disk_Info, NULL, sizeof(Disk_Info));

    /* create error log handle */
    DevCard_Error_Handle = ErrorLog.create("DevCard_Error");

    /* regist all error to the error tree */
    ErrorLog.registe(DevCard_Error_Handle, DevCard_ErrorList, sizeof(DevCard_ErrorList) / sizeof(DevCard_ErrorList[0]));

#if (STORAGE_MODULE & EXTERNAL_INTERFACE_TYPE_SPI_FLASH)
    Disk_Info.module_reg.section.FlashChip_module_EN = true;
    Disk_Info.module_error_reg.section.FlashChip_module_error_code = DevW25Q64.init(W25Q64_Obj);

    /* trigger error */
    if (Disk_Info.module_error_reg.section.FlashChip_module_error_code)
    {
        // ErrorLog.trigger(DevCard_Error_Handle, );
    }
#endif

#if (STORAGE_MODULE & EXTERNAL_INTERFACE_TYPE_TF_CARD)
    DevTFCard_Obj.SDMMC_Obj.pin = &SDMMC_Pin;

    Disk_Info.module_reg.section.TFCard_modlue_EN = true;
    Disk_Info.module_error_reg.section.TFCard_module_error_code = DevCard.Init(&DevTFCard_Obj.SDMMC_Obj);

    /* trigger error */
    if (Disk_Info.module_error_reg.section.TFCard_module_error_code)
    {
        uint8_t Error_Code = Disk_Info.module_error_reg.section.TFCard_module_error_code;

        ErrorLog.trigger(DevCard_Error_Handle,
                         DevCard_External_Module_Init_Error,
                         &Error_Code, sizeof(uint8_t));
    }
#endif

    if (Disk_Info.module_error_reg.val != 0)
        init_state = false;

    return init_state;
}

#if (STORAGE_MODULE & INTERNAL_INTERFACE_TYPE)
bool IntDisk_Init(void)
{
    bool init_state = false;

    return init_state;
}
#endif

void Disk_Set_Printf(Disk_Printf_Callback Callback)
{
    Disk_PrintOut = Callback;
}

bool Disk_Init(Disk_Printf_Callback Callback)
{
    memset(&Disk_Info, NULL, sizeof(Disk_Info));

    /* set printf callback */
    Disk_PrintOut = Callback;

#if (STORAGE_MODULE & INTERNAL_INTERFACE_TYPE)
    if (!IntDisk_Init())
        return false;
#endif

#if ((STORAGE_MODULE & EXTERNAL_INTERFACE_TYPE_TF_CARD) || (STORAGE_MODULE & EXTERNAL_INTERFACE_TYPE_SPI_FLASH))
    if (ExtDisk_Init() == STORAGE_MODULE_NO_ERROR)
        return false;
#endif

    Disk_Printf("Disk Init Done\r\n");

    return true;
}

/******************************************************************************* Error Proc Function **************************************************************************/
#if (STORAGE_MODULE & INTERNAL_INTERFACE_TYPE)
static void Disk_Internal_InitError(int16_t code, uint8_t *p_arg, uint16_t size)
{
}
#endif

static void Disk_ExtModule_InitError(int16_t code, uint8_t *p_arg, uint16_t size)
{
#if (STORAGE_MODULE & EXTERNAL_INTERFACE_TYPE_TF_CARD)
    Disk_Printf("\tTF Card Error Code: %d\r\n", *p_arg);
#endif
}

static void Disk_Printf(const char *str, ...)
{
    va_list arp;
    va_start(arp, str);

    if (Disk_PrintOut)
    {
        sprintf(Disk_Print_Buff, str, arp);
        Disk_PrintOut(Disk_Print_Buff, strlen(Disk_Print_Buff));
        memset(Disk_Print_Buff, NULL, sizeof(Disk_Print_Buff));
    }

    va_end(arp);
}