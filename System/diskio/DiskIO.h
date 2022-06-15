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

typedef DevCard_Info_TypeDef Disk_Card_Info;
typedef void (*Disk_Printf_Callback)(uint8_t *p_buff, uint16_t size);

typedef enum
{
    DevCard_Internal_Module_Init_Error = 0,
    DevCard_External_Module_Init_Error,
} DiskIO_Error_List;

typedef enum
{
    OCS_System_Param = 1,
    OCS_Applic_Param,
    OCS_Custom_Param,
} Default_Section_List;

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
    uint16_t ResdSecCnt;
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

    Disk_CardMBR_SectionInfo_TypeDef disk_section_table[DISK_CARD_SECTION_INFO_NUM];
    Disk_CardDBR_SectionInfo_TypeDef DBR_info;
} Disk_FATFileSys_TypeDef;
#pragma pack()

bool Disk_Init(Disk_Printf_Callback Callback);

#endif
