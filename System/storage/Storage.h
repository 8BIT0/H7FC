#ifndef __STORAGE_H
#define __STORAGE_H

#include "Srv_DataHub.h"
#include "Bsp_Flash.h"

#define OnChipFlash_Storage_StartAddress (FLASH_BASE_ADDR + FLASH_SECTOR_7_OFFSET_ADDR)
#define OnChipFlash_Stroage_TotalSize (FLASH_SECTOR_7_OFFSET_ADDR - FLASH_SECTOR_6_OFFSET_ADDR)

#define OnChipFlash_Storage_PageSize (1024 * 4)
#define OnChipFlash_Storage_InfoPageSize OnChipFlash_Storage_PageSize 

#define ExternalFlash_Storage_Address

#define INTERNAL_STORAGE_PAGE_TAG "[InternalFlash Storage]"
#define EXTERNAL_STORAGE_PAGE_TAG "[ExternalFlash Storage]"
#define INTERNAL_PAGE_TAG_SIZE sizeof(INTERNAL_STORAGE_PAGE_TAG)
#define EXTERNAL_PAGE_TAG_SIZE sizeof(EXTERNAL_STORAGE_PAGE_TAG)
#define STORAGE_TAG "DATA"
#define STORAGE_END_TAG 0xFF1001FF

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

typedef enum
{
    Stor_Folder = 0,
    Stor_File,
} Storage_FileType_List;

typedef struct
{
    uint16_t head_tag;
    uint16_t class_type;
    uint16_t sub_class_num;
    uint16_t sub_file_num;
    uint16_t file_type;
    uint8_t name[32];
    uint32_t next_addr;
    uint32_t size;
    uint16_t end_tag;
} Storage_MapTabItem_TypeDef;

typedef struct
{
    uint8_t tag[32];
    uint32_t free_block_addr;
    uint32_t total_stor_space;
    uint32_t inuse_stor_space;
    uint32_t unuse_stor_space;
    uint32_t map_start_addr;
    uint32_t map_page_num;
    uint32_t info_page_size;
} Storage_InfoPage_TypeDef;

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
    
    Storage_InfoPage_TypeDef IntStor_Info;
    Storage_InfoPage_TypeDef ExtStor_Info;
    
    bool init_state;
} Storage_Monitor_TypeDef;

typedef struct
{
    bool (*init)(Storage_ModuleState_TypeDef enable);
    storage_handle (*create_section)(uint32_t addr, uint16_t size);
    bool (*delete_section)(storage_handle hdl);
    bool (*save)(storage_handle hdl, uint8_t *p_data, uint16_t size);
    bool (*get)(storage_handle hdl, uint8_t *p_data, uint16_t size);
    bool (*clear)(storage_handle hdl);
} Storage_TypeDef;

extern Storage_TypeDef Storage;

#endif
