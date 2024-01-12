#ifndef __STORAGE_H
#define __STORAGE_H

#include <string.h>
#include "Srv_DataHub.h"
#include "Bsp_Flash.h"
#include "Srv_OsCommon.h"
#include "util.h"

#define Format_Retry_Cnt 5

#define OnChipFlash_Storage_StartAddress (FLASH_BASE_ADDR + FLASH_SECTOR_7_OFFSET_ADDR)
#define OnChipFlash_Storage_TotalSize FLASH_SECTOR_7_SIZE
#define OnChipFlash_Storage_DefaultData FLASH_DEFAULT_DATA

#define OnChipFlash_MaxRWSize (2 Kb)
#define OnChipFlash_Storage_TabSize (4 Kb)
#define OnChipFlash_Storage_InfoPageSize (1 Kb)

#define ExternalFlash_Storage_Address 0

#define From_Start_Address 0

#define BootSection_Block_Size (4 Kb)
#define BootTab_Num 1

#define Storage_Max_Capacity 256
#define Storage_ReserveBlock_Size 128

#define StorageItem_Size sizeof(Storage_Item_TypeDef)
#define Storage_Tab_MaxItem_Num (OnChipFlash_Storage_TabSize / StorageItem_Size)

#define INTERNAL_STORAGE_PAGE_TAG "[InternalFlash Storage]"
#define EXTERNAL_STORAGE_PAGE_TAG "[ExternalFlash Storage]"
#define INTERNAL_PAGE_TAG_SIZE strlen(INTERNAL_STORAGE_PAGE_TAG)
#define EXTERNAL_PAGE_TAG_SIZE strlen(EXTERNAL_STORAGE_PAGE_TAG)

#define STORAGE_NAME_LEN 53
#define STORAGE_ITEM_HEAD_TAG 0xAA
#define STORAGE_ITEM_END_TAG 0xBB
#define STORAGE_SLOT_HEAD_TAG 0xEF0110EF
#define STORAGE_SLOT_END_TAG 0xFE1001FE

#define STORAGE_MIN_BYTE_SIZE 1

typedef uint32_t storage_handle;

typedef enum
{
    Internal_Flash = 0,
    External_Flash,
} Storage_MediumType_List;

typedef enum
{
    Para_Boot = 0,
    Para_Sys,
    Para_User,
} Storage_ParaClassType_List;

typedef struct
{
    uint8_t head_tag;
    uint8_t class;
    uint8_t name[STORAGE_NAME_LEN];
    uint32_t data_addr;
    uint16_t len;
    uint16_t crc16;
    uint8_t end_tag;
} Storage_Item_TypeDef;

typedef struct
{
    uint32_t header;
    uint8_t name[STORAGE_NAME_LEN];
    uint32_t total_data_size;
    uint32_t cur_slot_size;
    uint8_t align_byte;
    /* storage data insert */
    uint16_t slot_crc;
    uint32_t ender;
} Storage_DataSlot_TypeDef;

typedef struct
{
    uint32_t header;
    uint32_t total_size;
    uint32_t cur_slot_size;
    uint32_t ender;
} Storage_FreeSlot_TypeDef;

typedef struct
{
    uint32_t tab_addr;
    uint32_t data_sec_addr;
    uint32_t data_sec_size;
    uint32_t page_num;
    uint32_t tab_size;
    uint32_t free_addr;
    uint32_t para_size;
    uint32_t para_num;
} Storage_BaseSecInfo_TypeDef;

typedef struct
{
    uint8_t tag[32];

    uint32_t total_size;
    uint32_t remain_size;
    uint32_t data_sec_size;

    Storage_BaseSecInfo_TypeDef boot_sec;
    Storage_BaseSecInfo_TypeDef sys_sec;
    Storage_BaseSecInfo_TypeDef user_sec;
} Storage_FlashInfo_TypeDef;

typedef union
{
    struct
    {
        uint8_t internal : 1;
        uint8_t res_1    : 3;

        uint8_t external : 1; 
        uint8_t res_2    : 3;
    } bit;

    uint8_t val;
} Storage_ModuleState_TypeDef;

typedef struct
{
    Storage_ModuleState_TypeDef module_enable_reg;
    Storage_ModuleState_TypeDef module_init_reg;

    uint8_t InternalFlash_Format_cnt;
    uint8_t ExternalFlash_Format_cnt;
    
    bool init_state;
    uint8_t inuse;

    Storage_FlashInfo_TypeDef internal_info;
    Storage_FlashInfo_TypeDef external_info;
} Storage_Monitor_TypeDef;

typedef struct
{
    bool (*init)(Storage_ModuleState_TypeDef enable);
    storage_handle (*search)(Storage_MediumType_List medium, Storage_ParaClassType_List class, const char *name);
    bool (*save)(storage_handle hdl, uint8_t *p_data, uint16_t size);
    bool (*get)(storage_handle hdl, uint8_t *p_data, uint16_t size);
    bool (*clear)(storage_handle hdl);
} Storage_TypeDef;

extern Storage_TypeDef Storage;

#endif
