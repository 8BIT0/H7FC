#include "DiskIO.h"
#include "Dev_W25Qxx.h"
#include "Dev_Card.h"
#include "IO_Definition.h"
#include "Data_Convert_Util.h"
#include "error_log.h"
#include <stdarg.h>
#include <stdio.h>

#pragma pack(1)
/* File and Folder Attribute definition */
typedef struct
{
    char name[8];
    char ext[3];

    uint8_t attr;
    uint8_t LowerCase;
    uint8_t Time10Ms;
    uint8_t CreateTime[2];
    uint8_t CreateDate[2];
    uint8_t AccessDate[2];
    uint8_t HighCluster[2];
    uint8_t ModifyTime[2];
    uint8_t ModifyDate[2];
    uint8_t LowCluster[2];
    uint8_t FileSize[4];
} Disk_FFAttr_TypeDef;

/* catlog cluster single section file/folder attribute table */
typedef struct
{
    Disk_FFAttr_TypeDef attribute[16];
} Disk_CCSSFFAT_TypeDef;
#pragma pack()

static uint8_t Disk_Print_Buff[128] = {0};
static Disk_Printf_Callback Disk_PrintOut = NULL;

static Error_Handler DevCard_Error_Handle = NULL;
static Disk_Info_TypeDef Disk_Info;

/* Internal Function */
static void Disk_ExtModule_InitError(int16_t code, uint8_t *p_arg, uint16_t size);
static void Disk_Printf(char *str, ...);

#if (STORAGE_MODULE & EXTERNAL_INTERFACE_TYPE_SPI_FLASH)
/******************************************************************************** SPI Interface **************************************************************************/
DevW25QxxObj_TypeDef W25Q64_Obj = {
    .bus_type = DevW25Qxx_Norm_SpiBus,
    .BusPort = SPI1,
};
#endif

#if (STORAGE_MODULE & EXTERNAL_INTERFACE_TYPE_TF_CARD)
static Disk_Card_Info Disk_GetCard_Info(void);
static void Disk_ParseMBR(Disk_FATFileSys_TypeDef *FATObj);
static void Disk_ParseDBR(Disk_FATFileSys_TypeDef *FATObj);
static char *Disk_GetFolderName_ByIndex(const char *fpath, uint32_t index);

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

static const uint8_t DiskCard_NoneMBR_Label[] = {0xEB, 0x58, 0x90};
static Disk_FATFileSys_TypeDef FATFs_Obj;

/* is not an appropriate data cache structure i think */
/* still developing */
static uint8_t Disk_Card_SectionBuff[DISK_CARD_SENCTION_SZIE * 2] = {0};

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
    memset(&FATFs_Obj, NULL, sizeof(FATFs_Obj));

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
    if (!ExtDisk_Init())
        return false;
#endif

    Disk_Printf("Disk Init Done\r\n");

#if (STORAGE_MODULE & EXTERNAL_INTERFACE_TYPE_TF_CARD)
    Disk_Printf("    [Card Info] BlockNbr:     %d\r\n", Disk_GetCard_Info().BlockNbr);
    Disk_Printf("    [Card Info] BlockSize:    %d\r\n", Disk_GetCard_Info().BlockSize);
    Disk_Printf("    [Card Info] CardSpeed:    %d\r\n", Disk_GetCard_Info().CardSpeed);
    Disk_Printf("    [Card Info] CardType:     %d\r\n", Disk_GetCard_Info().CardType);
    Disk_Printf("    [Card Info] CardVersion:  %d\r\n", Disk_GetCard_Info().CardVersion);
    Disk_Printf("    [Card Info] Class:        %d\r\n", Disk_GetCard_Info().Class);
    Disk_Printf("    [Card Info] LogBlockNbr:  %d\r\n", Disk_GetCard_Info().LogBlockNbr);
    Disk_Printf("    [Card Info] LogBlockSize: %d\r\n", Disk_GetCard_Info().LogBlockSize);
    Disk_Printf("    [Card Info] RelCardAdd:   %d\r\n", Disk_GetCard_Info().RelCardAdd);
    Disk_Printf("\r\n");

    /* Parse MBR Section Info */
    Disk_ParseMBR(&FATFs_Obj);
    Disk_ParseDBR(&FATFs_Obj);
#endif
    return true;
}

static Disk_Card_Info Disk_GetCard_Info(void)
{
    return DevCard.Get_Info(&DevTFCard_Obj.SDMMC_Obj);
}
/************************************************************************** Disk File Alloc Table Function ***************************************************************************/
#if (STORAGE_MODULE & EXTERNAL_INTERFACE_TYPE_TF_CARD)
static void Disk_ParseMBR(Disk_FATFileSys_TypeDef *FATObj)
{
    if (FATObj == NULL)
        return;

    DevCard.read(&DevTFCard_Obj.SDMMC_Obj, DISK_CARD_MBR_SECTION, Disk_Card_SectionBuff, DISK_CARD_SENCTION_SZIE, 1);

    if (memcmp(DiskCard_NoneMBR_Label, Disk_Card_SectionBuff, sizeof(DiskCard_NoneMBR_Label)) == 0)
    {
        FATObj->has_mbr = false;
    }
    else
    {
        /* Check TF Card Termination Byte */
        if ((*(Disk_Card_SectionBuff + DISK_CARD_MBR_TERMINATION_BYTE_1_OFFSET) == DISK_CARD_TERMINATION_BYTE_1) &&
            (*(Disk_Card_SectionBuff + DISK_CARD_MBR_TERMINATION_BYTE_2_OFFSET) == DISK_CARD_TERMINATION_BYTE_2))
        {
            FATObj->has_mbr = true;

            memcpy(&(FATObj->disk_section_table), Disk_Card_SectionBuff + DISK_CARD_MBR_STARTUP_OFFSET, DISK_CARD_SECTION_AREA_TABLE * DISK_CARD_SECTION_INFO_NUM);

            for (uint8_t sec_index = 0; sec_index < DISK_CARD_SECTION_INFO_NUM; sec_index++)
            {
                uint16_t StartCylSect = 0;
                uint32_t StartLBA = 0;
                uint32_t size = 0;

                StartCylSect = FATObj->disk_section_table[sec_index].StartCylSect;
                StartLBA = FATObj->disk_section_table[sec_index].StartLBA;
                size = FATObj->disk_section_table[sec_index].Size;

                FATObj->disk_section_table[sec_index].StartCylSect = LEndian2HalfWord(&StartCylSect);
                FATObj->disk_section_table[sec_index].StartLBA = LEndian2Word(&StartLBA);
                FATObj->disk_section_table[sec_index].Size = LEndian2Word(&size);
            }
        }

        memset(Disk_Card_SectionBuff, NULL, sizeof(Disk_Card_SectionBuff));
    }
}

static void Disk_ParseDBR(Disk_FATFileSys_TypeDef *FATObj)
{
    if (FATObj == NULL)
        return;

    if (FATObj->has_mbr)
    {
        DevCard.read(&DevTFCard_Obj.SDMMC_Obj, FATObj->disk_section_table[0].StartLBA, Disk_Card_SectionBuff, DISK_CARD_SENCTION_SZIE, 1);
    }
    else
    {
        /* if card has no MBR section then Read DBR info from the first section */
        DevCard.read(&DevTFCard_Obj.SDMMC_Obj, 0, Disk_Card_SectionBuff, DISK_CARD_SENCTION_SZIE, 1);
    }

    /* Check TF Card Termination Byte */
    if ((*(Disk_Card_SectionBuff + DISK_CARD_MBR_TERMINATION_BYTE_1_OFFSET) == DISK_CARD_TERMINATION_BYTE_1) &&
        (*(Disk_Card_SectionBuff + DISK_CARD_MBR_TERMINATION_BYTE_2_OFFSET) == DISK_CARD_TERMINATION_BYTE_2))
    {
        uint16_t half_word_tmp = 0;
        uint32_t word_tmp = 0;

        memcpy(&(FATObj->DBR_info), Disk_Card_SectionBuff, sizeof(FATObj->DBR_info));

        half_word_tmp = FATObj->DBR_info.BytesPerSec;
        FATObj->DBR_info.BytesPerSec = LEndian2HalfWord(&half_word_tmp);

        half_word_tmp = FATObj->DBR_info.RsvdSecCnt;
        FATObj->DBR_info.RsvdSecCnt = LEndian2HalfWord(&half_word_tmp);

        half_word_tmp = FATObj->DBR_info.RootEntCnt;
        FATObj->DBR_info.RootEntCnt = LEndian2HalfWord(&half_word_tmp);

        half_word_tmp = FATObj->DBR_info.TotSec16;
        FATObj->DBR_info.TotSec16 = LEndian2HalfWord(&half_word_tmp);

        half_word_tmp = FATObj->DBR_info.FATSz16;
        FATObj->DBR_info.FATSz16 = LEndian2HalfWord(&half_word_tmp);

        half_word_tmp = FATObj->DBR_info.SecPerTrk;
        FATObj->DBR_info.SecPerTrk = LEndian2HalfWord(&half_word_tmp);

        half_word_tmp = FATObj->DBR_info.NumHeads;
        FATObj->DBR_info.NumHeads = LEndian2HalfWord(&half_word_tmp);

        word_tmp = FATObj->DBR_info.HiddSec;
        FATObj->DBR_info.HiddSec = LEndian2Word(&word_tmp);

        word_tmp = FATObj->DBR_info.TotSec32;
        FATObj->DBR_info.TotSec32 = LEndian2Word(&word_tmp);

        word_tmp = FATObj->DBR_info.FATSz32;
        FATObj->DBR_info.FATSz32 = LEndian2Word(&word_tmp);

        half_word_tmp = FATObj->DBR_info.ExtFlags;
        FATObj->DBR_info.ExtFlags = LEndian2HalfWord(&half_word_tmp);

        half_word_tmp = FATObj->DBR_info.FSVer;
        FATObj->DBR_info.FSVer = LEndian2HalfWord(&half_word_tmp);

        word_tmp = FATObj->DBR_info.RootClus;
        FATObj->DBR_info.RootClus = LEndian2Word(&word_tmp);

        half_word_tmp = FATObj->DBR_info.FSInfo;
        FATObj->DBR_info.FSInfo = LEndian2HalfWord(&half_word_tmp);

        half_word_tmp = FATObj->DBR_info.BkBootSec;
        FATObj->DBR_info.BkBootSec = LEndian2HalfWord(&half_word_tmp);

        word_tmp = FATObj->DBR_info.VolID;
        FATObj->DBR_info.VolID = LEndian2Word(&word_tmp);

        FATObj->DBR_SecNo = FATObj->disk_section_table[0].StartLBA;
        FATObj->BytePerSection = FATObj->DBR_info.BytesPerSec;
        FATObj->FAT_Sections = FATObj->DBR_info.FATSz32;
        FATObj->SecPerCluster = FATObj->DBR_info.SecPerClus;
        FATObj->Fst_FATSector = FATObj->disk_section_table[0].StartLBA + FATObj->DBR_info.RsvdSecCnt;
        FATObj->Fst_DirSector = FATObj->Fst_FATSector + FATObj->DBR_info.NumFATs * FATObj->DBR_info.FATSz32;
        FATObj->Total_KBSize = FATObj->DBR_info.TotSec32;
    }

    memset(Disk_Card_SectionBuff, NULL, DISK_CARD_SENCTION_SZIE);
}

/* cluster number to section number */
static uint32_t Disk_Get_StartSectionOfCluster(Disk_FATFileSys_TypeDef *FATObj, FATCluster_Addr cluster)
{
    return (((cluster - 2) * FATObj->SecPerCluster) + FATObj->Fst_FATSector);
}

/* parse file/folder attribute */
static Dis_FFInfoTable_TypeDef Disk_Parse_Attribute(Disk_FATFileSys_TypeDef *FATObj, FATCluster_Addr cluster)
{
    Dis_FFInfoTable_TypeDef table_tmp;
    Disk_CCSSFFAT_TypeDef *attr_tmp = NULL;
    uint32_t date_tmp = 0;
    uint32_t disk_section_no = 0;
    char name[11] = {'\0'};

    memset(&table_tmp, NULL, sizeof(Dis_FFInfoTable_TypeDef));

    if (cluster != 0)
    {
        memset(Disk_Card_SectionBuff, NULL, DISK_CARD_SENCTION_SZIE);

        DevCard.read(&DevTFCard_Obj.SDMMC_Obj,
                     Disk_Get_StartSectionOfCluster(FATObj, cluster),
                     Disk_Card_SectionBuff, DISK_CARD_SENCTION_SZIE, 1);

        attr_tmp = (Disk_CCSSFFAT_TypeDef *)Disk_Card_SectionBuff;

        for (uint8_t i = 0; i < 16; i++)
        {
            memcpy(table_tmp.Info[i].name, attr_tmp->attribute[i].name, 11);
            table_tmp.Info[i].name[11] = '\0';

            table_tmp.Info[i].prop = attr_tmp->attribute[i].attr;
            table_tmp.Info[i].start_cluster = LEndian2HalfWord(attr_tmp->attribute[i].LowCluster) |
                                              (LEndian2HalfWord(attr_tmp->attribute[i].HighCluster) << 16);
            table_tmp.Info[i].size = LEndian2Word(attr_tmp->attribute[i].FileSize);

            /* parse create time */
            date_tmp = LEndian2HalfWord(attr_tmp->attribute[i].CreateTime);
            table_tmp.Info[i].create_time.sec = (date_tmp & DISK_FILE_DATE_SEC_MASK) * 2;
            date_tmp >>= DISK_FILE_DATE_SEC_BITS;

            table_tmp.Info[i].create_time.min = date_tmp & DISK_FILE_DATE_MIN_MASK;
            date_tmp >>= DISK_FILE_DATE_MIN_BITS;

            table_tmp.Info[i].create_time.hour = date_tmp & DISK_FILE_DATE_HOUR_MASK;
            table_tmp.Info[i].create_time.sec += (uint16_t)(attr_tmp->attribute[i].Time10Ms) / 100;

            /* parse create date */
            date_tmp = LEndian2HalfWord(attr_tmp->attribute[i].CreateDate);
            table_tmp.Info[i].create_date.day = date_tmp & DISK_FILE_DATE_DAY_MASK;
            date_tmp >>= DISK_FILE_DATE_DAY_BITS;

            table_tmp.Info[i].create_date.month = date_tmp & DISK_FILE_DATE_MONTH_MASK;
            date_tmp >>= DISK_FILE_DATE_MONTH_BITS;

            table_tmp.Info[i].create_date.year = date_tmp & DISK_FILE_DATE_YEAR_MASK;
            table_tmp.Info[i].create_date.year += DISK_FILE_DATEBASE_YEAR;

            /* parse modify time */
            date_tmp = LEndian2HalfWord(attr_tmp->attribute[i].ModifyTime);
            table_tmp.Info[i].modify_time.sec = (date_tmp & DISK_FILE_DATE_SEC_MASK) * 2;
            date_tmp >>= DISK_FILE_DATE_SEC_BITS;

            table_tmp.Info[i].modify_time.min = date_tmp & DISK_FILE_DATE_MIN_MASK;
            date_tmp >>= DISK_FILE_DATE_MIN_BITS;

            table_tmp.Info[i].modify_time.hour = date_tmp & DISK_FILE_DATE_HOUR_MASK;

            /* parse modify date */
            date_tmp = LEndian2HalfWord(attr_tmp->attribute[i].ModifyDate);
            table_tmp.Info[i].modify_date.day = date_tmp & DISK_FILE_DATE_DAY_MASK;
            date_tmp >>= DISK_FILE_DATE_DAY_BITS;

            table_tmp.Info[i].modify_date.month = date_tmp & DISK_FILE_DATE_MONTH_MASK;
            date_tmp >>= DISK_FILE_DATE_MONTH_BITS;

            table_tmp.Info[i].modify_date.year = date_tmp & DISK_FILE_DATE_YEAR_MASK;
            table_tmp.Info[i].modify_date.year += DISK_FILE_DATEBASE_YEAR;

            /* parse access date */
            date_tmp = LEndian2HalfWord(attr_tmp->attribute[i].AccessDate);
            table_tmp.Info[i].access_date.day = date_tmp & DISK_FILE_DATE_DAY_MASK;
            date_tmp >>= DISK_FILE_DATE_DAY_BITS;

            table_tmp.Info[i].access_date.month = date_tmp & DISK_FILE_DATE_MONTH_MASK;
            date_tmp >>= DISK_FILE_DATE_MONTH_BITS;

            table_tmp.Info[i].access_date.year = date_tmp & DISK_FILE_DATE_YEAR_MASK;
            table_tmp.Info[i].access_date.year += DISK_FILE_DATEBASE_YEAR;
        }

        memset(Disk_Card_SectionBuff, NULL, DISK_CARD_SENCTION_SZIE);
    }

    return table_tmp;
}

static DiskFATCluster_State_List Disk_GetCluster_State(FATCluster_Addr cluster)
{
    if (cluster == DISK_FAT_CLUSTER_IDLE_WORLD)
        return Disk_FATCluster_Idle;

    if (cluster == DISK_FAT_CLUSTER_BAD_WORLD)
        return Disk_FATCluster_Bad;

    if ((cluster >= DISK_FAT_CLUSTER_ALLOC_MIN_WORLD) &&
        (cluster <= DISK_FAT_CLUSTER_ALLOC_MAX_WORLD))
        return Disk_FATCluster_Alloc;

    if ((cluster >= DISK_FAT_CLUSTER_SYSRES_MIN_WORLD) &&
        (cluster <= DISK_FAT_CLUSTER_SYSRES_MAX_WORLD))
        return Disk_FATCluster_SysRes;

    if ((cluster >= DISK_FAT_CLUSTER_END_MIN_WORLD) &&
        (cluster <= DISK_FAT_CLUSTER_END_MAX_WORLD))
        return Disk_FATCluster_End;

    return Disk_FATCluster_Unknow;
}

/* it may has bug */
/* get cluster number from FAT table */
static FATCluster_Addr Disk_Get_NextCluster(Disk_FATFileSys_TypeDef *FATObj, FATCluster_Addr cluster)
{
    FATCluster_Addr clu_sec = 0;
    Disk_FAT_ItemTable_TypeDef *FAT_Table = NULL;
    FATCluster_Addr FATAddr_Tmp = 0;

    clu_sec = (cluster / DISK_FAT_CLUSTER_ITEM_SUM) + FATObj->Fst_FATSector;

    DevCard.read(&DevTFCard_Obj.SDMMC_Obj, clu_sec, Disk_Card_SectionBuff, DISK_CARD_SENCTION_SZIE, 1);

    FAT_Table = (Disk_FAT_ItemTable_TypeDef *)Disk_Card_SectionBuff;
    FATAddr_Tmp = FAT_Table->table_item[cluster % DISK_FAT_CLUSTER_ITEM_SUM];

    return LEndian2Word(&FATAddr_Tmp);
}

static FATCluster_Addr Disk_GetPath_StartCluster(const char *path, FATCluster_Addr cluster_in)
{
}

static uint32_t Disk_GetPath_Layer(const char *fpath)
{
    uint32_t layer = 0;

    while (strchr(fpath, DISK_FOLDER_TERMINATION) != 0)
    {
        layer++;
    }

    return layer;
}

static char *Disk_GetFolderName_ByIndex(const char *fpath, uint32_t index)
{
    char fpath_tmp[128] = {'\0'};
    char *token;
    uint32_t layer = index;

    if (strlen(fpath) > sizeof(fpath_tmp))
        return NULL;

    memcpy(fpath_tmp, fpath, strlen(fpath));

    token = strtok(fpath_tmp, DISK_FOLDER_TERMINATION);

    while (layer)
    {
        token = strtok(NULL, DISK_FOLDER_TERMINATION);

        if (token == NULL)
            return NULL;

        layer--;
    }

    return token;
}

static bool Disk_Create_Path(const char *fpath, const char *name)
{
    char *folder_name = NULL;
    char *folder_path_tmp = NULL;
    uint32_t folder_depth = 0;

    if (fpath == NULL)
        return false;

    folder_depth = Disk_GetPath_Layer(fpath);

    for (uint32_t i = 0; i < folder_depth; i++)
    {
        folder_name = Disk_GetFolderName_ByIndex(fpath, i);

        /* combine folder name to a path */
        folder_path_tmp = (folder_path_tmp, strcat(folder_name, DISK_FOLDER_TERMINATION));

        /* check corresponding folder exist or not in the first place */

        folder_name = NULL;
    }
}

static bool Disk_Create_File(const char *path, const char *name)
{
    /* check correspond file exist or not first */
}

static bool Disk_WriteToFile()
{
}

#endif
/******************************************************************************* Error Proc Function **************************************************************************/
#if (STORAGE_MODULE & INTERNAL_INTERFACE_TYPE)
static void Disk_Internal_InitError(int16_t code, uint8_t *p_arg, uint16_t size)
{
}
#endif

static void Disk_ExtModule_InitError(int16_t code, uint8_t *p_arg, uint16_t size)
{
#if (STORAGE_MODULE & EXTERNAL_INTERFACE_TYPE_TF_CARD)
    ErrorLog.add_desc("    TF Card Error Code: %d\r\n", *p_arg);
#endif
}

/********************************************************************************* Printf Interface ***************************************************************************/
static void Disk_Printf(char *str, ...)
{
    va_list arp;
    va_start(arp, str);

    if (Disk_PrintOut)
    {
        uint32_t length = vsnprintf((char *)Disk_Print_Buff, sizeof(Disk_Print_Buff), (char *)str, arp);
        Disk_PrintOut(Disk_Print_Buff, length);

        memset(Disk_Print_Buff, NULL, sizeof(Disk_Print_Buff));
    }

    va_end(arp);
}
