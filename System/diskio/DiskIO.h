#ifndef __DISKIO_H
#define __DISKIO_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "system_cfg.h"
#include "Dev_Card.h"

#define DISK_CARD_SENCTION_SZIE 512
#define DISK_CARD_MBR_SECTION 0
#define DISK_CARD_MBR_STARTUP_OFFSET 446
#define DISK_CARD_SECTION_AREA_TABLE 16
#define DISK_CARD_SECTION_INFO_NUM  4

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

typedef union
{
    struct
    {
        uint8_t Active : 1;
        uint8_t StartHead : 1;
        uint8_t StartCylSect : 2;
        uint8_t PartType : 1;
        uint8_t EndHead : 1;
        uint8_t EndCylSect : 2;
        uint8_t StartLBA : 4;
        uint8_t Size : 4;
    }reg_sec;

    uint16_t val;
}Disk_CardMBR_SectionInfo_TypeDef;

typedef struct
{
    bool has_mbr;

}Disk_FATFileSys_TypeDef;
#pragma pack()

bool Disk_Init(Disk_Printf_Callback Callback);

#endif
