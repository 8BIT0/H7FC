#ifndef __STORAGE_H
#define __STORAGE_H

#include "Srv_DataHub.h"
#include "Bsp_Flash.h"
#include "Srv_OsCommon.h"

#define Format_Retry_Cnt 5

#define OnChipFlash_Storage_StartAddress (FLASH_BASE_ADDR + FLASH_SECTOR_7_OFFSET_ADDR)
#define OnChipFlash_Storage_TotalSize FLASH_SECTOR_7_SIZE
#define OnChipFlash_Storage_DefaultData FLASH_DEFAULT_DATA

#define OnChipFlash_MaxRWSize (1024 * 2)
#define OnChipFlash_Storage_TabSize (1024 * 4)
#define OnChipFlash_Storage_InfoPageSize 1024

#define ExternalFlash_Storage_Address 0

#define From_Start_Address 0

#define BootSection_Block_Size (1024 * 4)
#define BootTab_Page_Num 2

#define INTERNAL_STORAGE_PAGE_TAG "[InternalFlash Storage]"
#define EXTERNAL_STORAGE_PAGE_TAG "[ExternalFlash Storage]"
#define INTERNAL_PAGE_TAG_SIZE strlen(INTERNAL_STORAGE_PAGE_TAG)
#define EXTERNAL_PAGE_TAG_SIZE strlen(EXTERNAL_STORAGE_PAGE_TAG)
#define STORAGE_HEAD_TAG 0xAA
#define STORAGE_END_TAG 0xBB
#define STORAGE_SECTION_END_TAG 0xFF1001FF

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
    uint8_t name[52];
    uint32_t data_addr;
    uint32_t next_data_addr;
    uint16_t len;
    uint8_t end_tag;
} Storage_Item_TypeDef;

typedef struct
{
    uint8_t tag[32];

    uint32_t boot_tab_addr;
    uint32_t boot_page_num;
    uint32_t boot_block_size;
    uint32_t boot_free_addr;
    uint32_t boot_para_size;
    uint32_t boot_para_num;

    uint32_t sys_tab_addr;
    uint32_t sys_page_num;
    uint32_t sys_block_size;
    uint32_t sys_free_addr;
    uint32_t sys_para_size;
    uint32_t sys_para_num;

    uint32_t user_tab_addr;
    uint32_t user_page_num;
    uint32_t user_block_size;
    uint32_t user_free_addr;
    uint32_t user_para_size;
    uint32_t user_para_num;
} Storage_SectionInfo_TypeDef;

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
} Storage_Monitor_TypeDef;

typedef struct
{
    bool (*init)(Storage_ModuleState_TypeDef enable);
    bool (*save)(storage_handle hdl, uint8_t *p_data, uint16_t size);
    bool (*get)(storage_handle hdl, uint8_t *p_data, uint16_t size);
    bool (*clear)(storage_handle hdl);
} Storage_TypeDef;

extern Storage_TypeDef Storage;

#endif
