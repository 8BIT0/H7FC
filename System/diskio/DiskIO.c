/*
 *  coder: 8_B!T0
 *  bref: TFCard data storage interface
 *  still has bug inside but it already can create folder and file
 */

/*
 *  error log still has some bug......wtf!?!
 */

#include "DiskIO.h"
#include "Dev_W25Qxx.h"
#include "Dev_Card.h"
#include "IO_Definition.h"
#include "Data_Convert_Util.h"
#include "error_log.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "IO_Definition.h"
#include "debug_util.h"

#define GEN_TIME(h, m, s) ((((uint16_t)h) << 11) + (((uint16_t)m) << 5) + (((uint16_t)s) >> 1))
#define GEN_DATE(y, m, d) (((((uint16_t)(y % 100)) + 20) << 9) + (((uint16_t)m) << 5) + ((uint16_t)d))

static uint8_t Disk_Print_Buff[128] = {0};
static Disk_Printf_Callback Disk_PrintOut = NULL;

static Error_Handler DevCard_Error_Handle = NULL;
static Disk_Info_TypeDef Disk_Info;

static uint32_t MIN_write_unit = 0;

/* eExternal Function */
static bool Disk_Init(Disk_FATFileSys_TypeDef *FATObj, Disk_Printf_Callback Callback);

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

static Disk_Card_Info Disk_GetCard_Info(void);
static void Disk_ParseMBR(Disk_FATFileSys_TypeDef *FATObj);
static void Disk_ParseDBR(Disk_FATFileSys_TypeDef *FATObj);
static void Disk_ParseFSINFO(Disk_FATFileSys_TypeDef *FATObj);
static bool Disk_Search_FreeCluster(Disk_FATFileSys_TypeDef *FATObj);
static FATCluster_Addr Disk_Open(Disk_FATFileSys_TypeDef *FATObj, const char *dir_path, const char *name, Disk_FileObj_TypeDef *FileObj);
static FATCluster_Addr Disk_Create_Folder(Disk_FATFileSys_TypeDef *FATObj, const char *name, FATCluster_Addr cluster);
static Disk_FileObj_TypeDef Disk_Create_File(Disk_FATFileSys_TypeDef *FATObj, const char *name, FATCluster_Addr cluster, uint32_t size);
static Disk_Write_State Disk_WriteData_ToFile(Disk_FATFileSys_TypeDef *FATObj, Disk_FileObj_TypeDef *FileObj, const uint8_t *p_data, uint16_t len);
static uint32_t Disk_Get_MinWriteByte(void);
static bool Disk_Update_File_Cluster(Disk_FileObj_TypeDef *FileObj, FATCluster_Addr cluster);
static void Disk_FileSize_Update(Disk_FileObj_TypeDef *FileObj);

/* Error Process Function */
static void Disk_FreeCluster_SearchError(int16_t code, uint8_t *p_arg, uint16_t size);
static void Disk_FSINFO_ReadError(int16_t code, uint8_t *p_arg, uint16_t size);

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

DevCard_Obj_TypeDef DevTFCard_Obj = {
    .SDMMC_Obj = {
        .instance = SDMMC1,
    },
};

static const uint8_t DiskCard_NoneMBR_Label[] = {0xEB, 0x58, 0x90};
static uint8_t Disk_Card_SectionBuff[DISK_CARD_SECTION_SZIE] __attribute__((section(".Perph_Section"))) = {0};
static uint8_t Disk_FileSection_DataCache[DISK_CARD_SECTION_SZIE] __attribute__((section(".Perph_Section"))) = {0};

#endif

DiskFS_TypeDef Disk = {
    .init = Disk_Init,
    .create_folder = Disk_Create_Folder,
    .create_file = Disk_Create_File,
    .open = Disk_Open,
    .write = Disk_WriteData_ToFile,
    .get_min_write_unit = Disk_Get_MinWriteByte,
};

/******************************************************************************* Error Proc Object **************************************************************************/
static Error_Obj_Typedef DevCard_ErrorList[] = {
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
    {
        .out = true,
        .log = false,
        .prc_callback = Disk_FSINFO_ReadError,
        .code = DevCard_Read_FSINFO_Error,
        .desc = "External Storage FSINFO Read Error\r\n",
        .proc_type = Error_Proc_Immd,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = Disk_FreeCluster_SearchError,
        .code = DevCard_No_FreeCluster,
        .desc = "External Storage No Free Cluster\r\n",
        .proc_type = Error_Proc_Immd,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
};

static bool ExtDisk_Init(Disk_FATFileSys_TypeDef *FATObj)
{
    bool init_state = true;

    memset(&Disk_Info, 0, sizeof(Disk_Info));

#if (STORAGE_MODULE & EXTERNAL_INTERFACE_TYPE_TF_CARD)
    memset(FATObj, 0, sizeof(FATObj));

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

#if (STORAGE_MODULE & EXTERNAL_INTERFACE_TYPE_SPI_FLASH)
    Disk_Info.module_reg.section.FlashChip_module_EN = true;
    Disk_Info.module_error_reg.section.FlashChip_module_error_code = DevW25Q64.init(W25Q64_Obj);

    /* trigger error */
    if (Disk_Info.module_error_reg.section.FlashChip_module_error_code)
    {
        // ErrorLog.trigger(DevCard_Error_Handle, );
    }
#endif

    if (Disk_Info.module_error_reg.val != 0)
        init_state = false;

    FATObj->init = init_state;

    return init_state;
}

static bool Disk_Init(Disk_FATFileSys_TypeDef *FATObj, Disk_Printf_Callback Callback)
{
    memset(&Disk_Info, 0, sizeof(Disk_Info));

    /* set printf callback */
    Disk_PrintOut = Callback;

#if ((STORAGE_MODULE & EXTERNAL_INTERFACE_TYPE_TF_CARD) || (STORAGE_MODULE & EXTERNAL_INTERFACE_TYPE_SPI_FLASH))
    /* create error log handle */
    DevCard_Error_Handle = ErrorLog.create("DevCard_Error");
    /* regist all error to the error tree */
    ErrorLog.registe(DevCard_Error_Handle, DevCard_ErrorList, sizeof(DevCard_ErrorList) / sizeof(DevCard_ErrorList[0]));

    if (!ExtDisk_Init(FATObj))
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
    Disk_ParseMBR(FATObj);
    Disk_ParseDBR(FATObj);
    Disk_ParseFSINFO(FATObj);
    Disk_Search_FreeCluster(FATObj);

    MIN_write_unit = Disk_GetCard_Info().BlockSize;
#endif
    return true;
}

static uint32_t Disk_Get_MinWriteByte(void)
{
    return MIN_write_unit;
}

/************************************************************************** Disk File Alloc Table Function ***************************************************************************/
#if (STORAGE_MODULE & EXTERNAL_INTERFACE_TYPE_TF_CARD)
static Disk_Card_Info Disk_GetCard_Info(void)
{
    return DevCard.Get_Info(&DevTFCard_Obj);
}

static void Disk_ParseMBR(Disk_FATFileSys_TypeDef *FATObj)
{
    if (FATObj == NULL)
        return;

    DevCard.read(&DevTFCard_Obj, DISK_CARD_MBR_SECTION, Disk_Card_SectionBuff, DISK_CARD_SECTION_SZIE, 1);

    if ((memcmp(DiskCard_NoneMBR_Label, Disk_Card_SectionBuff, sizeof(DiskCard_NoneMBR_Label)) == 0) ||
        (memcmp(Disk_Card_SectionBuff, NULL, sizeof(DiskCard_NoneMBR_Label)) == 0))
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

        memset(Disk_Card_SectionBuff, 0, sizeof(Disk_Card_SectionBuff));
    }
}

static void Disk_ParseDBR(Disk_FATFileSys_TypeDef *FATObj)
{
    if (FATObj == NULL)
        return;

    if (FATObj->has_mbr)
    {
        DevCard.read(&DevTFCard_Obj, FATObj->disk_section_table[0].StartLBA, Disk_Card_SectionBuff, DISK_CARD_SECTION_SZIE, 1);
    }
    else
    {
        /* if card has no MBR section then DBR info in the first section */
        DevCard.read(&DevTFCard_Obj, 0, Disk_Card_SectionBuff, DISK_CARD_SECTION_SZIE, 1);
    }

    /* Check TF Card Termination Byte */
    if ((*(Disk_Card_SectionBuff + DISK_CARD_MBR_TERMINATION_BYTE_1_OFFSET) == DISK_CARD_TERMINATION_BYTE_1) &&
        (*(Disk_Card_SectionBuff + DISK_CARD_MBR_TERMINATION_BYTE_2_OFFSET) == DISK_CARD_TERMINATION_BYTE_2))
    {
        uint16_t half_word_tmp = 0;
        uint32_t word_tmp = 0;

        memcpy(&(FATObj->DBR_info), Disk_Card_SectionBuff, sizeof(FATObj->DBR_info));

        half_word_tmp = FATObj->DBR_info.BytesPerSec;
        FATObj->DBR_info.BytesPerSec = LEndian2HalfWord((const uint8_t *)&half_word_tmp);

        half_word_tmp = FATObj->DBR_info.RsvdSecCnt;
        FATObj->DBR_info.RsvdSecCnt = LEndian2HalfWord((const uint8_t *)&half_word_tmp);

        half_word_tmp = FATObj->DBR_info.RootEntCnt;
        FATObj->DBR_info.RootEntCnt = LEndian2HalfWord((const uint8_t *)&half_word_tmp);

        half_word_tmp = FATObj->DBR_info.TotSec16;
        FATObj->DBR_info.TotSec16 = LEndian2HalfWord((const uint8_t *)&half_word_tmp);

        half_word_tmp = FATObj->DBR_info.FATSz16;
        FATObj->DBR_info.FATSz16 = LEndian2HalfWord((const uint8_t *)&half_word_tmp);

        half_word_tmp = FATObj->DBR_info.SecPerTrk;
        FATObj->DBR_info.SecPerTrk = LEndian2HalfWord((const uint8_t *)&half_word_tmp);

        half_word_tmp = FATObj->DBR_info.NumHeads;
        FATObj->DBR_info.NumHeads = LEndian2HalfWord((const uint8_t *)&half_word_tmp);

        word_tmp = FATObj->DBR_info.HiddSec;
        FATObj->DBR_info.HiddSec = LEndian2Word((const uint8_t *)&word_tmp);

        word_tmp = FATObj->DBR_info.TotSec32;
        FATObj->DBR_info.TotSec32 = LEndian2Word((const uint8_t *)&word_tmp);

        word_tmp = FATObj->DBR_info.FATSz32;
        FATObj->DBR_info.FATSz32 = LEndian2Word((const uint8_t *)&word_tmp);

        half_word_tmp = FATObj->DBR_info.ExtFlags;
        FATObj->DBR_info.ExtFlags = LEndian2HalfWord((const uint8_t *)&half_word_tmp);

        half_word_tmp = FATObj->DBR_info.FSVer;
        FATObj->DBR_info.FSVer = LEndian2HalfWord((const uint8_t *)&half_word_tmp);

        word_tmp = FATObj->DBR_info.RootClus;
        FATObj->DBR_info.RootClus = LEndian2Word((const uint8_t *)&word_tmp);

        half_word_tmp = FATObj->DBR_info.FSInfo;
        FATObj->DBR_info.FSInfo = LEndian2HalfWord((const uint8_t *)&half_word_tmp);

        half_word_tmp = FATObj->DBR_info.BkBootSec;
        FATObj->DBR_info.BkBootSec = LEndian2HalfWord((const uint8_t *)&half_word_tmp);

        word_tmp = FATObj->DBR_info.VolID;
        FATObj->DBR_info.VolID = LEndian2Word((const uint8_t *)&word_tmp);

        FATObj->DBR_SecNo = FATObj->disk_section_table[0].StartLBA;
        FATObj->BytePerSection = FATObj->DBR_info.BytesPerSec;
        FATObj->FAT_Sections = FATObj->DBR_info.FATSz32;
        FATObj->SecPerCluster = FATObj->DBR_info.SecPerClus;
        FATObj->Fst_FATSector = FATObj->DBR_SecNo + FATObj->DBR_info.RsvdSecCnt;
        FATObj->Fst_DirSector = FATObj->Fst_FATSector + FATObj->DBR_info.NumFATs * FATObj->DBR_info.FATSz32;
        FATObj->Total_KBSize = FATObj->DBR_info.TotSec32;
        FATObj->cluster_byte_size = FATObj->SecPerCluster * FATObj->BytePerSection;
    }

    memset(Disk_Card_SectionBuff, 0, DISK_CARD_SECTION_SZIE);
}

static void Disk_ParseFSINFO(Disk_FATFileSys_TypeDef *FATObj)
{
    bool error = true;
    Disk_CardFSINFO_Typedef FSInfo;
    uint32_t FSInfo_SecNo = 1;

    /* FSINFO section after DBR Section */
    if (FATObj == NULL)
        return;

    memset(&FSInfo, 0, sizeof(FSInfo));

    if (FATObj->has_mbr)
    {
        FSInfo_SecNo = FATObj->disk_section_table[0].StartLBA + 1;
        DevCard.read(&DevTFCard_Obj, FATObj->disk_section_table[0].StartLBA + 1, Disk_Card_SectionBuff, DISK_CARD_SECTION_SZIE, 1);
    }
    else
    {
        /* if card has no MBR section then FSInfo in the second section */
        DevCard.read(&DevTFCard_Obj, 1, Disk_Card_SectionBuff, DISK_CARD_SECTION_SZIE, 1);
    }

    memcpy(&FSInfo, Disk_Card_SectionBuff, DISK_CARD_SECTION_SZIE);

    /* check fsinfo frame right or not */
    if ((memcmp(FSInfo.header, DISK_CARD_FSINFO_HEADER, sizeof(FSInfo.header)) == 0) &&
        (memcmp(FSInfo.ender, DISK_CARD_FSINFO_ENDER, sizeof(FSInfo.ender)) == 0) &&
        (FSInfo.check_end[0] == DISK_CARD_TERMINATION_BYTE_1) &&
        (FSInfo.check_end[1] == DISK_CARD_TERMINATION_BYTE_2))
    {
        FATObj->FSInfo_SecNo = FSInfo_SecNo;
        FATObj->remain_cluster = LEndian2Word(FSInfo.remain_cluster);
        error = false;
    }
    else
        FATObj->FSInfo_SecNo = 0;

    if (error)
    {
        ErrorLog.trigger(DevCard_Error_Handle,
                         DevCard_Read_FSINFO_Error,
                         NULL, 0);
    }
}

static void Disk_UpdateFSINFO(Disk_FATFileSys_TypeDef *FATObj, uint32_t remain_clus)
{
    Disk_CardFSINFO_Typedef *FSInfo_Ptr = NULL;

    if ((FATObj == NULL) ||
        (FATObj->FSInfo_SecNo == 0) ||
        (remain_clus > (FATObj->DBR_info.TotSec32 - FATObj->DBR_info.FATSz32 * FATObj->DBR_info.NumFATs) / FATObj->BytePerSection))
        return;

    DevCard.read(&DevTFCard_Obj, FATObj->FSInfo_SecNo, Disk_Card_SectionBuff, DISK_CARD_SECTION_SZIE, 1);

    FSInfo_Ptr = Disk_Card_SectionBuff;
    LEndianWord2BytesArray(remain_clus, FSInfo_Ptr->remain_cluster);

    DevCard.write(&DevTFCard_Obj, FATObj->FSInfo_SecNo, Disk_Card_SectionBuff, DISK_CARD_SECTION_SZIE, 1);
}

/* cluster number to section number */
static uint32_t Disk_Get_StartSectionOfCluster(Disk_FATFileSys_TypeDef *FATObj, FATCluster_Addr cluster)
{
    if (cluster < ROOT_CLUSTER_ADDR)
        return 0;

    return (((cluster - ROOT_CLUSTER_ADDR) * FATObj->SecPerCluster) + FATObj->Fst_DirSector);
}

/* parse file/folder attribute */
static Disk_FFInfoTable_TypeDef Disk_Parse_Attribute(Disk_FATFileSys_TypeDef *FATObj, uint32_t sec)
{
    Disk_FFInfoTable_TypeDef table_tmp;
    Disk_CCSSFFAT_TypeDef *attr_tmp = NULL;
    uint32_t date_tmp = 0;
    char name[11];

    memset(name, '\0', sizeof(name));
    memset(&table_tmp, 0, sizeof(Disk_FFInfoTable_TypeDef));

    if (sec != 0)
    {
        memset(Disk_Card_SectionBuff, 0, DISK_CARD_SECTION_SZIE);

        DevCard.read(&DevTFCard_Obj, sec, Disk_Card_SectionBuff, DISK_CARD_SECTION_SZIE, 1);

        attr_tmp = (Disk_CCSSFFAT_TypeDef *)Disk_Card_SectionBuff;

        for (uint8_t i = 0; i < 16; i++)
        {
            memcpy(table_tmp.Info[i].name, attr_tmp->attribute[i].name, 8);
            memcpy(table_tmp.Info[i].name + 8, attr_tmp->attribute[i].ext, 3);

            table_tmp.Info[i].attr = attr_tmp->attribute[i].attr;
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

        memset(Disk_Card_SectionBuff, 0, DISK_CARD_SECTION_SZIE);
    }

    return table_tmp;
}

static bool Disk_Search_FreeCluster(Disk_FATFileSys_TypeDef *FATObj)
{
    if (FATObj == NULL)
        return false;

    for (uint32_t sec_index = 0; sec_index < FATObj->FAT_Sections; sec_index++)
    {
        DevCard.read(&DevTFCard_Obj, FATObj->Fst_FATSector + sec_index, Disk_Card_SectionBuff, DISK_CARD_SECTION_SZIE, 1);

        for (uint8_t item_index = 0; item_index < DISK_FAT_CLUSTER_ITEM_SUM; item_index++)
        {
            if (((Disk_FAT_ItemTable_TypeDef *)Disk_Card_SectionBuff)->table_item[item_index] == 0)
            {
                FATObj->free_cluster = sec_index * DISK_FAT_CLUSTER_ITEM_SUM + item_index;
                return true;
            }
        }
    }

    ErrorLog.trigger(DevCard_Error_Handle,
                     DevCard_No_FreeCluster,
                     NULL, 0);

    return false;
}

static DiskFATCluster_State_List Disk_GetClusterState(FATCluster_Addr cluster)
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

static bool Disk_isFolder(const uint8_t attr)
{
    if (attr & Disk_File_Sd)
        return true;

    return false;
}

/* get cluster number from FAT table */
static FATCluster_Addr Disk_Get_NextCluster(Disk_FATFileSys_TypeDef *FATObj, FATCluster_Addr cluster)
{
    FATCluster_Addr clu_sec = 0;
    Disk_FAT_ItemTable_TypeDef *FAT_Table = NULL;
    FATCluster_Addr FATAddr_Tmp = 0;

    clu_sec = (cluster / DISK_FAT_CLUSTER_ITEM_SUM) + FATObj->Fst_FATSector;

    DevCard.read(&DevTFCard_Obj, clu_sec, Disk_Card_SectionBuff, DISK_CARD_SECTION_SZIE, 1);

    FAT_Table = (Disk_FAT_ItemTable_TypeDef *)Disk_Card_SectionBuff;
    FATAddr_Tmp = FAT_Table->table_item[cluster % DISK_FAT_CLUSTER_ITEM_SUM];

    return LEndian2Word((const uint8_t *)&FATAddr_Tmp);
}

static uint32_t Disk_GetPath_Layer(const char *fpath)
{
    uint32_t layer = 0;
    char *fpath_remain = fpath;

    while (fpath_remain != 0)
    {
        fpath_remain = strchr(fpath_remain, DISK_FOLDER_STRTOK_MARK);

        if (fpath_remain == 0)
        {
            break;
        }
        else
        {
            layer++;
            fpath_remain += 1;
        }
    }

    return layer;
}

static bool Disk_GetFolderName_ByIndex(const char *fpath, uint32_t index, char *token)
{
    char *fpath_match = NULL;
    char *fpath_tmp = NULL;

    fpath_tmp = (char *)DISKIO_MALLOC(strlen(fpath) + 1);
    if (fpath_tmp == NULL)
    {
        DISKIO_FREE(fpath_tmp);
        return false;
    }

    fpath_match = (char *)DISKIO_MALLOC(strlen(fpath) + 1);
    if (fpath_match == NULL)
    {
        DISKIO_FREE(fpath_match);
        return false;
    }

    memset(fpath_match, '\0', strlen(fpath));
    memset(fpath_tmp, '\0', strlen(fpath));
    memcpy(fpath_tmp, fpath, strlen(fpath));

    fpath_match = strtok(fpath_tmp, DISK_FOLDER_STRTOK_SYMBOL);

    while (index)
    {
        memcpy(fpath_tmp, fpath, strlen(fpath));

        fpath_match = strtok(NULL, DISK_FOLDER_STRTOK_SYMBOL);

        if (fpath_match == NULL)
        {
            DISKIO_FREE(fpath_tmp);
            DISKIO_FREE(fpath_match);
            return false;
        }

        index--;
    }

    memcpy(token, fpath_match, strlen(fpath_match));

    DISKIO_FREE(fpath_tmp);
    DISKIO_FREE(fpath_match);
    return true;
}

/*
 *   match short file name funcution
 *   f_name: current file name
 *   m_name: match target file name
 */
static bool Disk_SFN_Match(char *f_name, char *m_name)
{
    char fn_tmp[11] = {'\0'};
    char mn_tmp[11] = {'\0'};

    memcpy(fn_tmp, f_name, 11);
    memcpy(mn_tmp, m_name, 11);

    if (memcmp(fn_tmp, mn_tmp, 11) != 0)
        return false;

    return true;
}

/*
 *  check SFN frame legal or not
 *  SFN short file name
 *   f_n SFN file name
 *   e_n SFN extend file name
 */
static bool Disk_SFN_LegallyCheck(char *f_name)
{
    char f_name_tmp[64] = {'\0'};
    char *f_n = NULL;
    char *e_n = NULL;
    uint8_t file_char_Ucase = 0;
    uint8_t file_char_Lcase = 0;
    uint8_t extend_char_Ucase = 0;
    uint8_t extend_char_Lcase = 0;
    const char illegal_letter_list[] = {'\\', '/', ':', '*', '?', '<', '>', '|', '"'};

    if ((f_name == NULL) || (strlen(f_name) > sizeof(f_name_tmp)))
        return false;

    memcpy(f_name_tmp, f_name, strlen(f_name));

    f_n = strtok(f_name_tmp, SFN_EXTEND_SPLIT_SYMBOL);

    if (f_n == NULL)
        return false;

    e_n = strtok(NULL, SFN_EXTEND_SPLIT_SYMBOL);

    /* step 1 file name size check 0 < f_n length <= 8 && 0 <= e_n length <= 3 */
    if ((strlen(f_n) > 0) &&
        (strlen(f_n) <= SFN_FILE_NAME_MAX_LENGTH) &&
        (strlen(e_n) >= 0) &&
        (strlen(e_n) <= SFN_EXTEND_NAME_MAX_LENGTH))
    {
        /* step 2 all Character legally check */
        for (uint8_t i = 0; i < sizeof(illegal_letter_list); i++)
        {
            if (strchr(e_n, illegal_letter_list[i]) != NULL)
                return false;

            if (strchr(f_n, illegal_letter_list[i]) != NULL)
                return false;
        }

        /* step 3 check Character case */
        for (uint8_t i = 0; i < SFN_FILE_NAME_MAX_LENGTH; i++)
        {
            if (i < SFN_EXTEND_NAME_MAX_LENGTH)
            {
                if (e_n != 0x00)
                {
                    if (isupper((int)e_n[i]))
                        extend_char_Ucase++;

                    if (islower((int)e_n[i]))
                        extend_char_Lcase++;
                }
            }

            if (f_n != 0x00)
            {
                if (isupper((int)f_n[i]))
                    file_char_Ucase++;

                if (islower((int)f_n[i]))
                    file_char_Lcase++;
            }
        }

        if (((file_char_Lcase == 0) && (extend_char_Lcase == 0)) || ((extend_char_Ucase == 0) && (file_char_Ucase == 0)))
            return true;
    }

    return false;
}

/*
 * convert input file name to 8 3 Frame Mode
 * 8 is file or folder name max name len is 8byte
 * 3 is extend file name max len is 3byte
 * ALL letter will convert to captial
 */
static bool Disk_Name_ConvertTo83Frame(const char *n_in, char *n_out)
{
    char n_in_tmp[12] = {'\0'};
    char *file_name_tmp = NULL;
    char *ext_name_tmp = NULL;
    char file_name[8];
    char ext_file_name[3];

    memcpy(n_in_tmp, n_in, strlen(n_in));
    memset(file_name, 0, sizeof(file_name));
    memset(ext_file_name, 0, sizeof(ext_file_name));

    if ((n_in_tmp == NULL) || (n_out == NULL) || !Disk_SFN_LegallyCheck(n_in_tmp))
        return false;

    file_name_tmp = strtok(n_in_tmp, SFN_EXTEND_SPLIT_SYMBOL);
    ext_name_tmp = strtok(NULL, SFN_EXTEND_SPLIT_SYMBOL);

    if (file_name_tmp == NULL)
        return false;

    for (uint8_t i = 0; i < sizeof(file_name); i++)
    {
        if (i < strlen(file_name_tmp))
        {
            if ((file_name_tmp[i] >= 'a') || (file_name_tmp[i] <= 'z'))
            {
                file_name[i] = toupper(file_name_tmp[i]);
            }
            else
                file_name[i] = file_name_tmp[i];
        }
        else
            file_name[i] = 0x20;

        n_out[i] = file_name[i];
    }

    for (uint8_t i = 0; i < sizeof(ext_file_name); i++)
    {
        if (i < strlen(ext_name_tmp))
        {
            if ((ext_name_tmp[i] >= 'a') || (ext_name_tmp[i] <= 'z'))
            {
                ext_file_name[i] = toupper(ext_name_tmp[i]);
            }
            else
                ext_file_name[i] = ext_name_tmp[i];
        }
        else
            ext_file_name[i] = 0x20;

        n_out[i + sizeof(file_name)] = ext_file_name[i];
    }

    return true;
}

static bool Disk_Fill_Attr(const char *name, Disk_StorageData_TypeDef type, Disk_FFAttr_TypeDef *Attr_Out, FATCluster_Addr cluster)
{
    char Name_Frame83[11];
    uint16_t half_cluster = 0;

    /* ptr check */
    if ((name == NULL) || (Attr_Out == NULL) ||
        /* file type check */
        (type < Disk_DataType_File) || (type > Disk_DataType_Folder) ||
        /* name legally check */
        !Disk_SFN_LegallyCheck(name))
        return false;

    memset(Name_Frame83, '\0', sizeof(Name_Frame83));

    Disk_Name_ConvertTo83Frame(name, Name_Frame83);
    memcpy(Attr_Out->name, Name_Frame83, sizeof(Name_Frame83));

    if (type == Disk_DataType_Folder)
    {
        Attr_Out->attr = Disk_File_Sd | Disk_File_RW | Disk_File_Pf;

        /* set cluster */
        half_cluster = cluster & 0xFFFF0000;
        half_cluster >>= 16;
        Attr_Out->HighCluster[0] = half_cluster;
        Attr_Out->HighCluster[1] = half_cluster >> 8;

        half_cluster = cluster & 0x0000FFFF;
        Attr_Out->LowCluster[0] = half_cluster;
        Attr_Out->LowCluster[1] = half_cluster >> 8;
    }
    else
        Attr_Out->attr = Disk_File_Pf;

    uint16_t t_date = GEN_DATE(DISK_FILE_DEFAULT_YEAR, DISK_FILE_DEFAULT_MONTH, DISK_FILE_DEFAULT_DAY);
    uint16_t t_time = GEN_TIME(DISK_FILE_DEFAULT_HOUR, DISK_FILE_DEFAULT_MIN, DISK_FILE_DEFAULT_SEC);

    Attr_Out->Time10Ms = 0;

    Attr_Out->CreateDate[0] = (uint8_t)t_date;
    Attr_Out->CreateDate[1] = (uint8_t)(t_date >> 8);

    Attr_Out->CreateTime[0] = (uint8_t)t_time;
    Attr_Out->CreateTime[1] = (uint8_t)(t_time >> 8);

    Attr_Out->ModifyDate[0] = (uint8_t)t_date;
    Attr_Out->ModifyDate[1] = (uint8_t)(t_date >> 8);

    Attr_Out->ModifyTime[0] = (uint8_t)t_time;
    Attr_Out->ModifyTime[1] = (uint8_t)(t_time >> 8);

    Attr_Out->LowerCase = 0x18;

    memset(Attr_Out->FileSize, 0, sizeof(Attr_Out->FileSize));

    return true;
}

static bool Disk_Establish_ClusterLink(Disk_FATFileSys_TypeDef *FATObj, const FATCluster_Addr cur_cluster, const FATCluster_Addr nxt_cluster)
{
    uint32_t sec_index = 0;
    uint32_t sec_item_index = 0;
    uint32_t *Data_Ptr = NULL;
    bool state = false;

    /* FAT1 equal to FAT2 */
    if ((FATObj == NULL) || (cur_cluster < ROOT_CLUSTER_ADDR) || (nxt_cluster < ROOT_CLUSTER_ADDR))
        return false;

    memset(Disk_Card_SectionBuff, 0, DISK_CARD_SECTION_SZIE);

    sec_index = FATObj->Fst_FATSector + (cur_cluster * sizeof(FATCluster_Addr)) / FATObj->BytePerSection;
    sec_item_index = (cur_cluster * sizeof(FATCluster_Addr)) % FATObj->BytePerSection;

    state = DevCard.read(&DevTFCard_Obj, sec_index, Disk_Card_SectionBuff, DISK_CARD_SECTION_SZIE, 1);

    if (!state)
        return false;

    Data_Ptr = (uint32_t *)&Disk_Card_SectionBuff[sec_item_index];
    LEndianWord2BytesArray(nxt_cluster, Data_Ptr);

    /* Update FAT1 Table */
    state = DevCard.write(&DevTFCard_Obj, sec_index, Disk_Card_SectionBuff, DISK_CARD_SECTION_SZIE, 1);

    if (!state)
        return false;

    /* Update FAT2 Table */
    sec_index += FATObj->FAT_Sections;
    // DevCard.read(&DevTFCard_Obj.SDMMC_Obj, sec_index, Disk_Card_SectionBuff, DISK_CARD_SECTION_SZIE, 1);

    Data_Ptr = (uint32_t)&Disk_Card_SectionBuff[sec_item_index];
    LEndianWord2BytesArray(nxt_cluster, Data_Ptr);

    state = DevCard.write(&DevTFCard_Obj, sec_index, Disk_Card_SectionBuff, DISK_CARD_SECTION_SZIE, 1);

    if (!state)
        return false;

    return true;
}

static bool Disk_ClearCluster(Disk_FATFileSys_TypeDef *FATObj, FATCluster_Addr target_cluster)
{
    uint32_t sec_id = 0;

    if (target_cluster < ROOT_CLUSTER_ADDR)
        return false;

    sec_id = Disk_Get_StartSectionOfCluster(FATObj, target_cluster);

    memset(Disk_Card_SectionBuff, 0, sizeof(Disk_Card_SectionBuff));

    for (uint8_t i = 0; i < FATObj->SecPerCluster; i++)
    {
        DevCard.write(&DevTFCard_Obj, sec_id, Disk_Card_SectionBuff, DISK_CARD_SECTION_SZIE, 1);
        sec_id++;
    }
    return true;
}

static bool Disk_Update_FreeCluster(Disk_FATFileSys_TypeDef *FATObj)
{
    uint32_t sec_index = 0;
    uint32_t cluster_tmp = 0;
    uint8_t free_index;

    if (FATObj == NULL)
        return false;

    if (FATObj->remain_cluster != 0)
    {
        FATObj->remain_cluster--;

        sec_index = FATObj->free_cluster / DISK_FAT_CLUSTER_ITEM_SUM;

        /* search new free cluster */
        DevCard.read(&DevTFCard_Obj, sec_index + FATObj->Fst_FATSector, Disk_Card_SectionBuff, DISK_CARD_SECTION_SZIE, 1);

        for (free_index = FATObj->free_cluster % DISK_FAT_CLUSTER_ITEM_SUM; free_index < DISK_FAT_CLUSTER_ITEM_SUM; free_index++)
        {
            cluster_tmp = LEndian2Word(&(((Disk_FAT_ItemTable_TypeDef *)Disk_Card_SectionBuff)->table_item[free_index]));

            if (cluster_tmp == 0)
            {
                FATObj->free_cluster = (sec_index * DISK_FAT_CLUSTER_ITEM_SUM) + free_index;

                /* update FSINFO section on TFCard */
                Disk_UpdateFSINFO(FATObj, FATObj->remain_cluster);
                return true;
            }
        }

        /* search free cluster from the behind */
        for (uint32_t nxt_sec = sec_index + 1; nxt_sec < FATObj->FAT_Sections; nxt_sec++)
        {
            DevCard.read(&DevTFCard_Obj, nxt_sec + FATObj->Fst_FATSector, Disk_Card_SectionBuff, DISK_CARD_SECTION_SZIE, 1);

            for (free_index = 0; free_index < DISK_FAT_CLUSTER_ITEM_SUM; free_index++)
            {
                cluster_tmp = LEndian2Word(&(((Disk_FAT_ItemTable_TypeDef *)Disk_Card_SectionBuff)->table_item[free_index]));

                if (cluster_tmp == 0)
                {
                    FATObj->free_cluster = (nxt_sec * DISK_FAT_CLUSTER_ITEM_SUM) + free_index;

                    /* update FSINFO section on TFCard */
                    Disk_UpdateFSINFO(FATObj, FATObj->remain_cluster);
                    return true;
                }
            }
        }
    }

    FATObj->free_cluster = ROOT_CLUSTER_ADDR;
    return false;
}

static FATCluster_Addr Disk_WriteTo_TargetFFTable(Disk_FATFileSys_TypeDef *FATObj, Disk_StorageData_TypeDef type, const char *name, FATCluster_Addr cluster)
{
    uint32_t sec_id = 0;
    FATCluster_Addr target_file_cluster = cluster;
    FATCluster_Addr lst_cluster = 0;
    DiskFATCluster_State_List Cluster_State = Disk_GetClusterState(target_file_cluster);
    Disk_FFInfoTable_TypeDef FFInfo;
    Disk_FFAttr_TypeDef attr_tmp;
    char file_name[12];
    char folder_name[9];
    char *name_tmp;
    bool state = false;

    if (type != Disk_DataType_Folder)
    {
        name_tmp = file_name;
        memset(name_tmp, '\0', 12);
        memcpy(name_tmp, name, 12);
    }
    else
    {
        name_tmp = folder_name;
        memset(name_tmp, '\0', 9);
        memcpy(name_tmp, name, 9);
    }

    if (!Disk_Name_ConvertTo83Frame(name_tmp, name_tmp))
        return 0;

    while (Cluster_State == Disk_FATCluster_Alloc)
    {
        sec_id = Disk_Get_StartSectionOfCluster(FATObj, target_file_cluster);

        for (uint8_t section_index = 0; section_index < FATObj->SecPerCluster; section_index++)
        {
            sec_id += section_index;
            memset(&FFInfo, 0, sizeof(FFInfo));

            FFInfo = Disk_Parse_Attribute(FATObj, sec_id);

            for (uint8_t FF_index = 0; FF_index < 16; FF_index++)
            {
                if (FFInfo.Info[FF_index].name[0] != '\0')
                {
                    if (Disk_SFN_Match(FFInfo.Info[FF_index].name, name_tmp))
                    {
                        if(FFInfo.Info[FF_index].start_cluster)
                            return FFInfo.Info[FF_index].start_cluster;
                        // return target_file_cluster;
                    }
                }
                else
                {
                    if (type != Disk_DataType_Folder)
                    {
                        /* create file */
                        memset(name_tmp, '\0', 12);
                        memcpy(name_tmp, name, 12);
                    }
                    else
                    {
                        memset(name_tmp, '\0', 9);
                        memcpy(name_tmp, name, 9);
                    }

                    memset(&attr_tmp, 0, sizeof(Disk_FFAttr_TypeDef));
                    Disk_Fill_Attr(name_tmp, type, &attr_tmp, FATObj->free_cluster);

                    /* read all section data first */
                    DevCard.read(&DevTFCard_Obj, sec_id, Disk_Card_SectionBuff, DISK_CARD_SECTION_SZIE, 1);

                    /* corver current index of data */
                    memcpy(&(((Disk_CCSSFFAT_TypeDef *)Disk_Card_SectionBuff)->attribute[FF_index]), &attr_tmp, sizeof(attr_tmp));

                    /* write back to tf section */
                    DevCard.write(&DevTFCard_Obj, sec_id, Disk_Card_SectionBuff, sizeof(Disk_CCSSFFAT_TypeDef), 1);
                    memset(Disk_Card_SectionBuff, 0, sizeof(Disk_CCSSFFAT_TypeDef));

                    if (type == Disk_DataType_Folder)
                    {
                        state = Disk_Establish_ClusterLink(FATObj, FATObj->free_cluster, DISK_FAT_CLUSTER_END_MIN_WORLD);
                        while (!state)
                        {
                            state = false;
                        }

                        Disk_ClearCluster(FATObj, FATObj->free_cluster);

                        memset(attr_tmp.name, ' ', 8);
                        memset(attr_tmp.ext, ' ', 3);
                        attr_tmp.name[0] = '.';

                        memcpy(Disk_Card_SectionBuff, &attr_tmp, sizeof(Disk_FFAttr_TypeDef));

                        attr_tmp.name[1] = '.';
                        target_file_cluster = FATObj->free_cluster;

                        attr_tmp.LowCluster[0] = target_file_cluster;
                        attr_tmp.LowCluster[1] = target_file_cluster >> 8;

                        attr_tmp.HighCluster[0] = target_file_cluster >> 16;
                        attr_tmp.HighCluster[1] = target_file_cluster >> 24;

                        memcpy(Disk_Card_SectionBuff + sizeof(Disk_FFAttr_TypeDef), &attr_tmp, sizeof(Disk_FFAttr_TypeDef));

                        sec_id = Disk_Get_StartSectionOfCluster(FATObj, FATObj->free_cluster);
                        state = DevCard.write(&DevTFCard_Obj, sec_id, Disk_Card_SectionBuff, sizeof(Disk_Card_SectionBuff), 1);

                        while (!state)
                        {
                            state = false;
                        }

                        Disk_Update_FreeCluster(FATObj);
                        Disk_ClearCluster(FATObj, FATObj->free_cluster);
                    }

                    return target_file_cluster;
                }
            }
        }

        lst_cluster = target_file_cluster;
        target_file_cluster = Disk_Get_NextCluster(FATObj, target_file_cluster);
        Cluster_State = Disk_GetClusterState(target_file_cluster);
    }

    /* if all inuse cluster have no space to write in then create new a new cluster if we have some free */
    if (FATObj->remain_cluster)
    {
        Disk_Establish_ClusterLink(FATObj, lst_cluster, FATObj->free_cluster);
        Disk_Establish_ClusterLink(FATObj, FATObj->free_cluster, DISK_FAT_CLUSTER_END_MAX_WORLD);
        Disk_ClearCluster(FATObj, FATObj->free_cluster);

        /* create file */
        memset(name_tmp, '\0', 12);
        memcpy(name_tmp, name, strlen(name));
        memset(&attr_tmp, 0, sizeof(Disk_FFAttr_TypeDef));

        Disk_Fill_Attr(name_tmp, type, &attr_tmp, target_file_cluster);
        sec_id = Disk_Get_StartSectionOfCluster(FATObj, FATObj->free_cluster);

        /* read all section data first */
        DevCard.read(&DevTFCard_Obj, sec_id, Disk_Card_SectionBuff, DISK_CARD_SECTION_SZIE, 1);

        /* corver current index of data */
        memcpy(&(((Disk_CCSSFFAT_TypeDef *)Disk_Card_SectionBuff)->attribute[0]), &attr_tmp, sizeof(attr_tmp));

        /* write back to tf section */
        DevCard.write(&DevTFCard_Obj, sec_id, Disk_Card_SectionBuff, sizeof(Disk_CCSSFFAT_TypeDef), 1);

        // update new free cluster
        Disk_Update_FreeCluster(FATObj);
        Disk_ClearCluster(FATObj, FATObj->free_cluster);

        if (type == Disk_DataType_Folder)
        {
            Disk_Establish_ClusterLink(FATObj, FATObj->free_cluster, DISK_FAT_CLUSTER_END_MAX_WORLD);
            Disk_ClearCluster(FATObj, FATObj->free_cluster);

            memset(((Disk_CCSSFFAT_TypeDef *)Disk_Card_SectionBuff)->attribute[0].name, 0, sizeof(((Disk_CCSSFFAT_TypeDef *)Disk_Card_SectionBuff)->attribute[0].name));
            memset(((Disk_CCSSFFAT_TypeDef *)Disk_Card_SectionBuff)->attribute[0].ext, 0, sizeof(((Disk_CCSSFFAT_TypeDef *)Disk_Card_SectionBuff)->attribute[0].ext));
            ((Disk_CCSSFFAT_TypeDef *)Disk_Card_SectionBuff)->attribute[0].name[0] = '.';

            memset(Disk_Card_SectionBuff, 0, DISK_CARD_SECTION_SZIE);
            memcpy(Disk_Card_SectionBuff, &(((Disk_CCSSFFAT_TypeDef *)Disk_Card_SectionBuff)->attribute[0]), sizeof(Disk_FFAttr_TypeDef));

            ((Disk_CCSSFFAT_TypeDef *)Disk_Card_SectionBuff)->attribute[0].name[1] = '.';

            if (target_file_cluster > ROOT_CLUSTER_ADDR)
            {
                ((Disk_CCSSFFAT_TypeDef *)Disk_Card_SectionBuff)->attribute[0].HighCluster[0] = target_file_cluster >> 16;
                ((Disk_CCSSFFAT_TypeDef *)Disk_Card_SectionBuff)->attribute[0].HighCluster[1] = target_file_cluster >> 24;
                ((Disk_CCSSFFAT_TypeDef *)Disk_Card_SectionBuff)->attribute[0].LowCluster[0] = target_file_cluster;
                ((Disk_CCSSFFAT_TypeDef *)Disk_Card_SectionBuff)->attribute[0].LowCluster[1] = target_file_cluster >> 8;
            }
            else
            {
                ((Disk_CCSSFFAT_TypeDef *)Disk_Card_SectionBuff)->attribute[0].HighCluster[0] = 0;
                ((Disk_CCSSFFAT_TypeDef *)Disk_Card_SectionBuff)->attribute[0].HighCluster[1] = 0;
                ((Disk_CCSSFFAT_TypeDef *)Disk_Card_SectionBuff)->attribute[0].LowCluster[0] = 0;
                ((Disk_CCSSFFAT_TypeDef *)Disk_Card_SectionBuff)->attribute[0].LowCluster[1] = 0;
            }

            memcpy(Disk_Card_SectionBuff + sizeof(Disk_FFAttr_TypeDef), &(((Disk_CCSSFFAT_TypeDef *)Disk_Card_SectionBuff)->attribute[0]), sizeof(Disk_FFAttr_TypeDef));

            sec_id = Disk_Get_StartSectionOfCluster(FATObj, FATObj->free_cluster);
            DevCard.write(&DevTFCard_Obj, sec_id, Disk_Card_SectionBuff, sizeof(Disk_CCSSFFAT_TypeDef), 1);

            target_file_cluster = FATObj->free_cluster;

            // update new free cluster
            Disk_Update_FreeCluster(FATObj);
            Disk_ClearCluster(FATObj, FATObj->free_cluster);
        }
    }

    return 0;
}

static Disk_TargetMatch_TypeDef Disk_MatchTaget(Disk_FATFileSys_TypeDef *FATObj, char *name, Disk_StorageData_TypeDef type, Disk_FFInfo_TypeDef *F_Info, FATCluster_Addr start_cluster)
{
    FATCluster_Addr cluster_tmp = start_cluster;
    Disk_TargetMatch_TypeDef match_state = {0};
    uint32_t sec_id = Disk_Get_StartSectionOfCluster(FATObj, cluster_tmp);
    DiskFATCluster_State_List Cluster_State = Disk_GetClusterState(cluster_tmp);
    Disk_FFInfoTable_TypeDef FFInfo;
    char SFN_name_tmp[11] = {'\0'};

    match_state.match = false;

    if ((name == NULL) || (type > Disk_DataType_Folder) || !Disk_Name_ConvertTo83Frame(name, SFN_name_tmp))
        return match_state;

    while (Cluster_State == Disk_FATCluster_Alloc)
    {
        for (uint8_t i = 0; i < FATObj->SecPerCluster; i++)
        {
            memset(&FFInfo, 0, sizeof(FFInfo));

            FFInfo = Disk_Parse_Attribute(FATObj, sec_id + i);

            for (uint8_t j = 0; j < 16; j++)
            {
                /* match search type and short file name */
                if ((Disk_isFolder(FFInfo.Info[j].attr) == type) &&
                    (DISK_DELETED_MARK != FFInfo.Info[j].name[0]) &&
                    Disk_SFN_Match(FFInfo.Info[j].name, SFN_name_tmp))
                {
                    memcpy(F_Info, &(FFInfo.Info[j]), sizeof(Disk_FFInfo_TypeDef));

                    match_state.match = true;
                    match_state.sec_index = sec_id + i;
                    match_state.info_index = j;
                    match_state.cluster_index = cluster_tmp;

                    return match_state;
                }
            }
        }

        cluster_tmp = Disk_Get_NextCluster(FATObj, cluster_tmp);
        Cluster_State = Disk_GetClusterState(cluster_tmp);
        sec_id = Disk_Get_StartSectionOfCluster(FATObj, cluster_tmp);
    }

    match_state.match = false;
    return match_state;
}

/*
 * max sub folder deepth 3 layer
 * for exp "dir1/dir2/dir3/"
 */
static FATCluster_Addr Disk_Create_Folder(Disk_FATFileSys_TypeDef *FATObj, const char *name, FATCluster_Addr cluster)
{
    char name_tmp[9];
    uint32_t layer = 0;
    uint32_t name_index = 0;
    FATCluster_Addr cluster_tmp = cluster;
    FATCluster_Addr lst_cluster = cluster;
    Disk_TargetMatch_TypeDef match_state;
    Disk_FFInfo_TypeDef F_Info;

    memset(&F_Info, 0, sizeof(F_Info));
    memset(&match_state, 0, sizeof(match_state));

    /* name is folder path break it down 1st */
    layer = Disk_GetPath_Layer(name);

    for (name_index = 0; name_index < layer; name_index++)
    {
        memset(name_tmp, '\0', sizeof(name_tmp));

        /* search any same name item has exist 2nd */
        if (Disk_GetFolderName_ByIndex(name, name_index, name_tmp))
        {
            /* name legally check */
            if (!Disk_SFN_LegallyCheck(name_tmp))
                return 0;

            cluster_tmp = Disk_WriteTo_TargetFFTable(FATObj, Disk_DataType_Folder, name_tmp, cluster_tmp);

            if (cluster_tmp == 0)
                return 0;

            /* check if created successful */
            match_state = Disk_MatchTaget(FATObj, name_tmp, Disk_DataType_Folder, &F_Info, lst_cluster);

            if (!match_state.match || (lst_cluster != match_state.cluster_index))
                return 0;

            if (name_index == layer - 1)
                return cluster_tmp;

            lst_cluster = cluster_tmp;
        }
        else
            return 0;

        memset(name_tmp, '\0', (strlen(name) + 1));
    }

    return 0;
}

/* optimize return file object */
static Disk_FileObj_TypeDef Disk_Create_File(Disk_FATFileSys_TypeDef *FATObj, const char *file, FATCluster_Addr cluster, uint32_t size)
{
    Disk_FileObj_TypeDef file_tmp;
    Disk_FFInfo_TypeDef F_Info;
    Disk_TargetMatch_TypeDef match_state;
    uint32_t exp_cluster_cnt;
    item_obj *cluster_list_item_tmp = NULL;
    Disk_PreLinkBlock_TypeDef *cluster_id_ptr = NULL;

    memset(&match_state, 0, sizeof(match_state));
    memset(&F_Info, 0, sizeof(F_Info));
    memset(&file_tmp, 0, sizeof(file_tmp));

    if ((file != NULL) && (strlen(file) < 12))
    {
        if (Disk_WriteTo_TargetFFTable(FATObj, Disk_DataType_File, file, cluster))
        {
            match_state = Disk_MatchTaget(FATObj, file, Disk_DataType_File, &F_Info, cluster);

            if (match_state.match)
            {
                memset(Disk_Card_SectionBuff, 0, DISK_CARD_SECTION_SZIE);
                DevCard.read(&DevTFCard_Obj, match_state.sec_index, Disk_Card_SectionBuff, DISK_CARD_SECTION_SZIE, 1);

                memcpy(&file_tmp.info, &(((Disk_CCSSFFAT_TypeDef *)Disk_Card_SectionBuff)->attribute[match_state.info_index]), sizeof(file_tmp.info));

                /* set data to default */
                file_tmp.cursor_pos = 0;
                file_tmp.info_index = match_state.info_index;
                file_tmp.remain_byte_in_sec = FATObj->BytePerSection;
                file_tmp.info_sec = match_state.sec_index;
                file_tmp.end_sec = 0;

                if (size)
                {
                    /* comput cluster we need */
                    exp_cluster_cnt = size / FATObj->cluster_byte_size;
                    file_tmp.fast_mode = true;
                    file_tmp.total_byte_remain = size;
                    file_tmp.info.size = size;

                    if (exp_cluster_cnt)
                    {
                        Disk_Printf("    [Prepare File]\r\n");
                        Disk_Printf("    [File Info] FileName:           %s\r\n", file);
                        Disk_Printf("    [File Info] Requir Cluster Num: %d\r\n", exp_cluster_cnt);

                        /* create linked list item first */
                        cluster_list_item_tmp = (item_obj *)DISKIO_MALLOC(sizeof(item_obj));
                        /* create linked list */
                        cluster_id_ptr = (Disk_PreLinkBlock_TypeDef *)DISKIO_MALLOC(sizeof(Disk_PreLinkBlock_TypeDef));
                        /* init list item */
                        List_ItemInit(cluster_list_item_tmp, cluster_id_ptr);
                        List_Init(&file_tmp.cluster_list, cluster_list_item_tmp, by_order, NULL);
                        Disk_Update_File_Cluster(&file_tmp, FATObj->free_cluster);
                        if (cluster_list_item_tmp && cluster_id_ptr)
                        {
                            FATCluster_Addr start_cluster = FATObj->free_cluster;
                            FATCluster_Addr end_cluster = 0;
                            FATCluster_Addr lst_end_cluster = end_cluster;

                            cluster_id_ptr->s_addr = start_cluster;
                            Disk_Establish_ClusterLink(FATObj, start_cluster, DISK_FAT_CLUSTER_END_MIN_WORLD);

                            /* search free cluster and link them */
                            for (uint32_t i = 0; i < exp_cluster_cnt; i++)
                            {
                                Disk_Update_FreeCluster(FATObj);
                                end_cluster = FATObj->free_cluster;

                                /* link up cluster */
                                /* File Cluster -> Free Cluster 1 -> Free Cluster 2 -> ... -> Free Cluster end */
                                Disk_Establish_ClusterLink(FATObj, start_cluster, end_cluster);
                                Disk_Establish_ClusterLink(FATObj, end_cluster, DISK_FAT_CLUSTER_END_MIN_WORLD);

                                start_cluster = end_cluster;

                                /* create new lisnked list item */
                                if (FATObj->free_cluster - end_cluster > 1)
                                {
                                    cluster_list_item_tmp = (item_obj *)DISKIO_MALLOC(sizeof(item_obj));
                                    cluster_id_ptr = (Disk_PreLinkBlock_TypeDef *)DISKIO_MALLOC(sizeof(Disk_PreLinkBlock_TypeDef));

                                    if (cluster_list_item_tmp && cluster_id_ptr)
                                    {
                                        List_ItemInit(cluster_list_item_tmp, cluster_id_ptr);
                                        List_Insert_Item(&file_tmp.cluster_list, cluster_list_item_tmp);

                                        cluster_id_ptr->s_addr = end_cluster;
                                        cluster_id_ptr->e_addr = end_cluster;
                                    }
                                    else
                                    {
                                        Disk_Establish_ClusterLink(FATObj, FATObj->free_cluster, DISK_FAT_CLUSTER_END_MIN_WORLD);
                                        file_tmp.fast_mode = false;

                                        DISKIO_FREE(cluster_id_ptr);
                                        DISKIO_FREE(cluster_list_item_tmp);
                                    }
                                }
                                else
                                    /* get free cluster id */
                                    cluster_id_ptr->e_addr = end_cluster;
                            }

                            if (file_tmp.fast_mode)
                            {
                                /* update file size */
                                Disk_FileSize_Update(&file_tmp);
                                file_tmp.info.size = 0;

                                memcpy(&file_tmp.cur_cluster_item, &file_tmp.cluster_list, sizeof(item_obj));
                            }
                        }
                        else
                            file_tmp.fast_mode = false;
                    }
                }
            }
        }
    }

    return file_tmp;
}

static FATCluster_Addr Disk_Get_DirStartCluster(Disk_FATFileSys_TypeDef *FATObj, const char *dir)
{
    Disk_FFInfo_TypeDef F_Info;
    uint16_t dir_layer = 0;
    FATCluster_Addr cluster_tmp = 2;
    uint16_t dir_size = strlen(dir) + 1;
    bool match = false;
    char *dir_tmp = NULL;

    dir_tmp = (char *)DISKIO_MALLOC(dir_size);
    memset(&F_Info, 0, sizeof(F_Info));

    if ((dir == NULL) || (dir_tmp == NULL))
    {
        DISKIO_FREE(dir_tmp);
        return 0;
    }

    /* direction break down first */
    dir_layer = Disk_GetPath_Layer(dir);
    memset(dir_tmp, '\0', dir_size);

    if (dir_layer)
    {
        for (uint32_t l = 0; l < dir_layer; l++)
        {
            match = false;
            Disk_GetFolderName_ByIndex(dir, l, dir_tmp);

            if (Disk_MatchTaget(FATObj, dir_tmp, Disk_DataType_Folder, &F_Info, cluster_tmp).match)
            {
                match = true;
                cluster_tmp = F_Info.start_cluster;
            }

            memset(dir_tmp, 0, strlen(dir_tmp));
        }
    }

    if (!match || (dir_layer == 0))
        cluster_tmp = 0;

    DISKIO_FREE(dir_tmp);

    return cluster_tmp;
}

static bool Disk_Update_File_Cluster(Disk_FileObj_TypeDef *FileObj, FATCluster_Addr cluster)
{
    Disk_FFAttr_TypeDef *attr_tmp = NULL;

    if ((FileObj == NULL) || (cluster < ROOT_CLUSTER_ADDR))
        return false;

    DevCard.read(&DevTFCard_Obj, FileObj->info_sec, Disk_Card_SectionBuff, DISK_CARD_SECTION_SZIE, 1);

    attr_tmp = &(((Disk_CCSSFFAT_TypeDef *)Disk_Card_SectionBuff)->attribute[FileObj->info_index]);

    attr_tmp->HighCluster[0] = cluster >> 16;
    attr_tmp->HighCluster[1] = cluster >> 24;
    attr_tmp->LowCluster[0] = cluster;
    attr_tmp->LowCluster[1] = cluster >> 8;

    DevCard.write(&DevTFCard_Obj, FileObj->info_sec, Disk_Card_SectionBuff, DISK_CARD_SECTION_SZIE, 1);
    FileObj->info.start_cluster = cluster;

    return true;
}

static void Disk_FileSize_Update(Disk_FileObj_TypeDef *FileObj)
{
    /* update file size */
    DevCard.read(&(DevTFCard_Obj), FileObj->info_sec, Disk_Card_SectionBuff, DISK_CARD_SECTION_SZIE, 1);
    LEndianWord2BytesArray(FileObj->info.size, ((Disk_CCSSFFAT_TypeDef *)Disk_Card_SectionBuff)->attribute[FileObj->info_index].FileSize);
    DevCard.write(&DevTFCard_Obj, FileObj->info_sec, Disk_Card_SectionBuff, DISK_CARD_SECTION_SZIE, 1);
}

/* write into an empty file */
static bool Disk_WriteFile_From_Head(Disk_FATFileSys_TypeDef *FATObj, Disk_FileObj_TypeDef *FileObj, const uint8_t *p_data, uint16_t len)
{
    // uint32_t use_cluster = 0;
    // uint32_t use_sec = 0;
    // FATCluster_Addr lst_file_cluster = 0;

    if ((FATObj == NULL) || (FileObj == NULL) || (p_data == NULL) || (len == 0))
        return false;
    
    if(!FileObj->fast_mode)
        Disk_Update_File_Cluster(FileObj, FATObj->free_cluster);

    // use_cluster = len / FATObj->cluster_byte_size;

    /* has bug in write in cluster */
    // for (uint16_t i = 0; i < use_cluster; i++)
    // {
    //     lst_file_cluster = FileObj->info.start_cluster;
    //     FileObj->info.start_cluster = FATObj->free_cluster;
    //     FileObj->end_sec = Disk_Get_StartSectionOfCluster(FATObj, FileObj->info.start_cluster);
    //     Disk_Update_FreeCluster(FATObj);

    //     DevCard.write(&DevTFCard_Obj.SDMMC_Obj, FileObj->end_sec, Disk_FileSection_DataCache, DISK_CARD_SECTION_SZIE, FATObj->SecPerCluster);
    //     Disk_Establish_ClusterLink(FATObj, lst_file_cluster, FileObj->info.start_cluster);
    // }

    /* write remain data info target file cluster */
    if (len % FATObj->cluster_byte_size)
    {
        // lst_file_cluster = FileObj->info.start_cluster;
        // FileObj->start_sec = FileObj->end_sec;
        FileObj->end_sec = Disk_Get_StartSectionOfCluster(FATObj, FileObj->info.start_cluster);

        // use_sec = (len % FATObj->cluster_byte_size) / FATObj->BytePerSection;

        // if (use_sec)
        // {
        //     memset(Disk_FileSection_DataCache, '\0', DISK_CARD_SECTION_SZIE);
        //     memcpy(Disk_FileSection_DataCache, p_data, len);

        //     DevCard.write(&DevTFCard_Obj.SDMMC_Obj, FileObj->end_sec, p_data, DISK_CARD_SECTION_SZIE, use_sec);
        //     FileObj->end_sec += use_sec;
        // }

        if (len % FATObj->BytePerSection)
        {
            memset(Disk_FileSection_DataCache, '\0', DISK_CARD_SECTION_SZIE);
            memcpy(Disk_FileSection_DataCache, p_data, len);

            DevCard.write(&DevTFCard_Obj, FileObj->end_sec, Disk_FileSection_DataCache, len % FATObj->BytePerSection, 1);
        }
    }

    if(!FileObj->fast_mode)
    {
        Disk_Establish_ClusterLink(FATObj, FileObj->info.start_cluster, DISK_FAT_CLUSTER_END_MAX_WORLD);
        Disk_Update_FreeCluster(FATObj);
    }
    
    Disk_ClearCluster(FATObj, FATObj->free_cluster);

    FileObj->info.size = len;
    FileObj->cursor_pos = len;
    FileObj->remain_byte_in_sec -= len;

    if(!FileObj->fast_mode)
    {
        /* update file size */
        Disk_FileSize_Update(FileObj);
    }

    return true;
}

/* need measure the cast of operation down below */
static Disk_Write_State Disk_WriteData_ToFile(Disk_FATFileSys_TypeDef *FATObj, Disk_FileObj_TypeDef *FileObj, const uint8_t *p_data, uint16_t len)
{
    uint16_t write_len = 0;
    uint16_t remain_write = 0;
    uint16_t base_len = len;
    uint32_t cluster_end_section = 0;

    if ((FATObj == NULL) || (!FATObj->init) || (FileObj == NULL) || (p_data == NULL) || (len == 0) || (FileObj->cursor_pos > FATObj->BytePerSection))
        return Disk_Write_Error;

    if (memcmp(FileObj->info.name, 0, sizeof(FileObj->info.name)) == 0)
        return Disk_Write_Error;

    if (FileObj->info.size == 0)
    {
        if (Disk_WriteFile_From_Head(FATObj, FileObj, p_data, len))
            return Disk_Write_Contiguous;

        return Disk_Write_Error;
    }

    cluster_end_section = Disk_Get_StartSectionOfCluster(FATObj, FileObj->info.start_cluster) + FATObj->SecPerCluster;

    do
    {
        if (FileObj->remain_byte_in_sec >= len)
        {
            write_len = len;

            if (remain_write)
            {
                p_data += base_len - remain_write;
                memcpy(Disk_FileSection_DataCache + FileObj->cursor_pos, p_data, remain_write);

                FileObj->remain_byte_in_sec -= remain_write;
                FileObj->cursor_pos += remain_write;
                FileObj->cursor_pos %= FATObj->BytePerSection;
                FileObj->info.size += remain_write;

                remain_write = 0;
            }

            len = 0;
        }
        else
        {
            write_len = FileObj->remain_byte_in_sec;
            remain_write = base_len - FileObj->remain_byte_in_sec;
            len -= FileObj->remain_byte_in_sec;
        }

        if (FileObj->fast_mode)
        {
            if (FileObj->total_byte_remain >= write_len)
            {
                FileObj->total_byte_remain -= write_len;
                FileObj->info.size += write_len;
            }
            else
                return Disk_Write_Finish;
        }

        memcpy(Disk_FileSection_DataCache + FileObj->cursor_pos, p_data, write_len);

        FileObj->remain_byte_in_sec -= write_len;
        FileObj->cursor_pos += write_len;
        FileObj->cursor_pos %= FATObj->BytePerSection;

        if (FileObj->remain_byte_in_sec == 0)
        {
            // DebugPin.ctl(Debug_PB4, true);
            DevCard.write(&DevTFCard_Obj, FileObj->end_sec, Disk_FileSection_DataCache, DISK_CARD_SECTION_SZIE, 1);
            // DebugPin.ctl(Debug_PB4, false);

            memset(Disk_FileSection_DataCache, '\0', DISK_CARD_SECTION_SZIE);

            if (FileObj->end_sec == cluster_end_section)
            {
                if (!FileObj->fast_mode)
                {
                    FATCluster_Addr lst_file_cluster = 0;

                    lst_file_cluster = FileObj->info.start_cluster;

                    Disk_Update_FreeCluster(FATObj);

                    if ((FileObj->info.start_cluster == FATObj->free_cluster) || (FATObj->free_cluster <= ROOT_CLUSTER_ADDR))
                        while (1) // error occur fall into infinite loop
                            ;

                    FileObj->info.start_cluster = FATObj->free_cluster;

                    /* establish cluster link */
                    Disk_Establish_ClusterLink(FATObj, lst_file_cluster, FileObj->info.start_cluster);
                    Disk_Establish_ClusterLink(FATObj, FileObj->info.start_cluster, DISK_FAT_CLUSTER_END_MIN_WORLD);
                }
                else
                {
                    FileObj->info.start_cluster++;
                    if (FileObj->info.start_cluster > ((Disk_PreLinkBlock_TypeDef *)(FileObj->cur_cluster_item.data))->e_addr)
                    {
                        if (FileObj->cur_cluster_item.nxt != NULL)
                        {
                            memcpy(&FileObj->cur_cluster_item, FileObj->cur_cluster_item.nxt, sizeof(item_obj));

                            FileObj->info.start_cluster = ((Disk_PreLinkBlock_TypeDef *)(FileObj->cur_cluster_item.data))->s_addr;
                        }
                        else
                        {
                            FileObj->info.start_cluster--;
                            return Disk_Write_Finish;
                        }
                    }
                }

                /* update end section */
                FileObj->end_sec = Disk_Get_StartSectionOfCluster(FATObj, FileObj->info.start_cluster);
                cluster_end_section = FileObj->end_sec + FATObj->SecPerCluster;
            }

            /* update end section */
            FileObj->end_sec++;
            FileObj->remain_byte_in_sec = FATObj->BytePerSection;
        }

        if (!FileObj->fast_mode)
            Disk_FileSize_Update(FileObj);

    } while (remain_write != 0);

    return Disk_Write_Contiguous;
}

static FATCluster_Addr Disk_Open(Disk_FATFileSys_TypeDef *FATObj, const char *dir_path, const char *name, Disk_FileObj_TypeDef *FileObj)
{
    char *name_buff;
    FATCluster_Addr file_cluster = ROOT_CLUSTER_ADDR;
    uint16_t name_size = strlen(name) + 1;
    Disk_TargetMatch_TypeDef match_state = {0};
    // uint32_t file_read_sec = 0;

    name_buff = (char *)DISKIO_MALLOC(name_size);
    if (name_buff == NULL)
    {
        DISKIO_FREE(name_buff);
        return 0;
    }

    if (dir_path != NULL)
    {
        /* match dir first get path cluster */
        file_cluster = Disk_Get_DirStartCluster(FATObj, dir_path);

        if (file_cluster == 0)
            return 0;
    }

    if (name == NULL)
        return file_cluster;

    memset(name_buff, '\0', name_size);
    memcpy(name_buff, name, name_size);

    match_state = Disk_MatchTaget(FATObj, name_buff, Disk_DataType_File, &(FileObj->info), file_cluster);
    if (match_state.match)
    {
        if (FileObj->fast_mode)
        {
            FileObj->info.start_cluster = ((Disk_PreLinkBlock_TypeDef *)(FileObj->cur_cluster_item.data))->s_addr;
            FileObj->end_sec = Disk_Get_StartSectionOfCluster(FATObj, FileObj->info.start_cluster);
        }
        else
        {

            FileObj->info_index = match_state.info_index;
            // FileObj->start_sec = match_state.sec_index;
            FileObj->end_sec = match_state.sec_index;

            /* comput cursor pos */
            FileObj->cursor_pos = /*FileObj->info.size*/ 0;

            memset(Disk_FileSection_DataCache, 0, sizeof(Disk_FileSection_DataCache));
            FileObj->info_sec = Disk_Get_StartSectionOfCluster(FATObj, file_cluster);

            if (FileObj->info.size)
            {
                /* update file last data section to cache data */
            }
        }

        DISKIO_FREE(name_buff);
        return FileObj->info.start_cluster;
    }

    DISKIO_FREE(name_buff);
    return 0;
}

#endif
/******************************************************************************* Error Proc Function **************************************************************************/

static void Disk_ExtModule_InitError(int16_t code, uint8_t *p_arg, uint16_t size)
{
#if (STORAGE_MODULE & EXTERNAL_INTERFACE_TYPE_TF_CARD)
    ErrorLog.add_desc("    TF Card Error Code: %d\r\n", *p_arg);
#endif
}

#if (STORAGE_MODULE & EXTERNAL_INTERFACE_TYPE_TF_CARD)
static void Disk_FreeCluster_SearchError(int16_t code, uint8_t *p_arg, uint16_t size)
{
    ErrorLog.add_desc("    No Free Cluster Be Found\r\n");
}

static void Disk_FSINFO_ReadError(int16_t code, uint8_t *p_arg, uint16_t size)
{
    ErrorLog.add_desc("    No FSINFO Section Be Found\r\n");
}

#endif

/********************************************************************************* Printf Interface ***************************************************************************/
static void Disk_Printf(char *str, ...)
{
    va_list arp;
    va_start(arp, str);

    if (Disk_PrintOut)
    {
        uint32_t length = vsnprintf((char *)Disk_Print_Buff, sizeof(Disk_Print_Buff), (char *)str, arp);
        Disk_PrintOut(Disk_Print_Buff, length);

        memset(Disk_Print_Buff, 0, sizeof(Disk_Print_Buff));
    }

    va_end(arp);
}
