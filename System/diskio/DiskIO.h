#ifndef __DISKIO_H
#define __DISKIO_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "system_cfg.h"
#include "Dev_Card.h"

#define DISK_CARD_SENCTION_SZIE 512
#define DISK_CARD_MBR_TERMINATION_BYTE_1_OFFSET 510
#define DISK_CARD_MBR_TERMINATION_BYTE_2_OFFSET 511
#define DISK_CARD_MBR_SECTION 0
#define DISK_CARD_MBR_STARTUP_OFFSET 446
#define DISK_CARD_SECTION_AREA_TABLE 16
#define DISK_CARD_SECTION_INFO_NUM 4
#define DISK_CARD_TERMINATION_BYTE_1 0x55
#define DISK_CARD_TERMINATION_BYTE_2 0xAA
#define DISK_CARD_DBR_OEM_OFFSET 3
#define DISK_FAT_CLUSTER_ITEM_SUM 128

#define DISK_FAT_CLUSTER_IDLE_WORLD 0x00000000
#define DISK_FAT_CLUSTER_ALLOC_MIN_WORLD 0x00000002
#define DISK_FAT_CLUSTER_ALLOC_MAX_WORLD 0x0FFFFFEF
#define DISK_FAT_CLUSTER_SYSRES_MIN_WORLD 0x0FFFFFF0
#define DISK_FAT_CLUSTER_SYSRES_MAX_WORLD 0x0FFFFFF6
#define DISK_FAT_CLUSTER_BAD_WORLD 0x0FFFFFF7
#define DISK_FAT_CLUSTER_END_MIN_WORLD 0x0FFFFFF8
#define DISK_FAT_CLUSTER_END_MAX_WORLD 0x0FFFFFFF

#define DISK_FILE_DATEBASE_YEAR 1980
#define DISK_FILE_DATE_HOUR_MASK 0x001F
#define DISK_FILE_DATE_MIN_MASK 0x002F
#define DISK_FILE_DATE_SEC_MASK 0x001F
#define DISK_FILE_DATE_YEAR_MASK 0x007F
#define DISK_FILE_DATE_MONTH_MASK 0x000F
#define DISK_FILE_DATE_DAY_MASK 0x001F

#define DISK_FILE_DATE_HOUR_BITS 5
#define DISK_FILE_DATE_MIN_BITS 6
#define DISK_FILE_DATE_SEC_BITS 5
#define DISK_FILE_DATE_YEAR_BITS 7
#define DISK_FILE_DATE_MONTH_BITS 4
#define DISK_FILE_DATE_DAY_BITS 5

typedef uint32_t FATCluster_Addr;
typedef DevCard_Info_TypeDef Disk_Card_Info;
typedef void (*Disk_Printf_Callback)(uint8_t *p_buff, uint16_t size);

typedef enum
{
    DevCard_Internal_Module_Init_Error = 0,
    DevCard_External_Module_Init_Error,
    DevCard_Read_MBR_Error,
    DevCard_Read_DBR_Error,
} DiskIO_Error_List;

typedef enum
{
    OCS_System_Param = 1,
    OCS_Applic_Param,
    OCS_Custom_Param,
} Default_Section_List;

typedef enum
{
    Disk_File_RW = 0b00000000, /* file attribute read & write */
    Disk_File_RO = 0b00000001, /* file attribute read only */
    Disk_File_Hi = 0b00000010, /* file attribute hide */
    Disk_File_Sy = 0b00000100, /* file attribute system file */
    Disk_File_Vo = 0b00001000, /* file attribute volumn tag */
    Disk_File_Sd = 0b00010000, /* file attribute subdirectory */
    Disk_File_Pf = 0b00100000, /* file attribute palce on file */
} Disk_FileAttr_List;

typedef enum
{
    Card_FSType_FAT32 = 0x01,
    Card_FSType_Win95_FAT32_1 = 0x0B,
    Card_FSType_Win95_FAT32_2 = 0x0C,
    Card_FSType_Hidden_FAT32 = 0x1B,
} DiskCard_FAT_TypeList;

typedef enum
{
    Disk_FATCluster_Unknow = 0,
    Disk_FATCluster_Idle,
    Disk_FATCluster_Alloc,
    Disk_FATCluster_SysRes,
    Disk_FATCluster_Bad,
    Disk_FATCluster_End,
} DiskFATCluster_State_List;

#pragma pack(1)
typedef union
{
    struct
    {
        uint8_t internal_module_EN : 1;
        uint8_t TFCard_modlue_EN : 1;
        uint8_t FlashChip_module_EN : 1;
        uint8_t reserve : 1;

        uint8_t FlashChip_module_CNT : 4;
    } section;

    uint8_t val;
} StorageModule_TypeDef;

typedef union
{
    struct
    {
        uint8_t internal_module_error_code : 8;
        uint8_t TFCard_module_error_code : 8;
        uint8_t FlashChip_module_error_code : 8;
        uint8_t reserve : 8;
    } section;

    uint32_t val;
} StorageModule_Error_Reg;

typedef struct
{
    FATCluster_Addr table_item[DISK_FAT_CLUSTER_ITEM_SUM];
} Disk_FAT_ItemTable_TypeDef;

typedef struct
{
    StorageModule_TypeDef module_reg;
    StorageModule_Error_Reg module_error_reg;
} Disk_Info_TypeDef;

typedef struct
{
    uint8_t Active;
    uint8_t StartHead;
    uint16_t StartCylSect;
    uint8_t PartType;
    uint8_t EndHead;
    uint16_t EndCylSect;
    uint32_t StartLBA;
    uint32_t Size;
} Disk_CardMBR_SectionInfo_TypeDef;

typedef struct
{
    uint8_t JMP_CMD[3];
    char OEM_tag[8];

    uint16_t BytesPerSec;
    uint8_t SecPerClus;
    uint16_t RsvdSecCnt;
    uint8_t NumFATs;
    uint16_t RootEntCnt;
    uint16_t TotSec16;
    uint8_t Media;
    uint16_t FATSz16;
    uint16_t SecPerTrk;
    uint16_t NumHeads;
    uint32_t HiddSec;
    uint32_t TotSec32;
    uint32_t FATSz32;
    uint16_t ExtFlags;
    uint16_t FSVer;
    uint32_t RootClus;
    uint16_t FSInfo;
    uint16_t BkBootSec;
    uint8_t Reserved[12];
    uint8_t DrvNum;
    uint8_t Reserved1;
    uint8_t BootSig;
    uint32_t VolID;
    uint8_t FileSysType[11];
    uint64_t FilSysType1;
} Disk_CardDBR_SectionInfo_TypeDef;

typedef struct
{
    bool has_mbr;
    bool has_dbr;

    Disk_CardMBR_SectionInfo_TypeDef disk_section_table[DISK_CARD_SECTION_INFO_NUM];
    Disk_CardDBR_SectionInfo_TypeDef DBR_info;

    uint32_t Fst_FATSector;
    uint32_t Fst_DirSector;
} Disk_FATFileSys_TypeDef;

typedef struct
{
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
} Disk_FileTime_TypeDef;

typedef struct
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
} Disk_FileDate_TypeDef;

typedef struct
{
    char *name;
    uint8_t prop;

    Disk_FileTime_TypeDef ctreate_time;
    Disk_FileDate_TypeDef create_date;

    Disk_FileDate_TypeDef access_date;

    Disk_FileTime_TypeDef modify_time;
    Disk_FileDate_TypeDef modify_date;

    uint32_t start_cluster;
    uint32_t size;
} Disk_FileInfo_TypeDef;
#pragma pack()

bool Disk_Init(Disk_Printf_Callback Callback);

#endif
