#ifndef __STORAGE_H
#define __STORAGE_H

#include "Srv_DataHub.h"
#include "Bsp_Flash.h"
#include "Srv_OsCommon.h"

#define OnChipFlash_Storage_StartAddress (FLASH_BASE_ADDR + FLASH_SECTOR_7_OFFSET_ADDR)
#define OnChipFlash_Stroage_TotalSize FLASH_SECTOR_7_SIZE

#define OnChipFlash_Storage_TabSize (1024 * 4)
#define OnChipFlash_Storage_InfoPageSize OnChipFlash_Storage_TabSize 

#define ExternalFlash_Storage_Address 0

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

typedef struct
{
    uint8_t head_tag;
    uint8_t name[20];
    uint32_t data_addr;
    uint32_t next_data_addr;
    uint16_t len;
    uint8_t end_tag;
} Storage_Item_TypeDef;

typedef struct
{
    uint8_t tag[32];

    uint32_t boot_para_addr;
    uint32_t boot_free_addr;
    uint32_t boot_para_size;

    uint32_t sys_para_addr;
    uint32_t sys_free_addr;
    uint32_t sys_para_size;

    uint32_t user_para_addr;
    uint32_t user_free_addr;
    uint32_t user_para_size;

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
