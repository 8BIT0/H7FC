#ifndef __STORAGE_H
#define __STORAGE_H

#include "Srv_DataHub.h"
#include "Bsp_Flash.h"

#define OnChipFlash_Storage_StartAddress 
#define OnChipFlash_Stroage_TotalSize

#define OnChipFlash_Storage_PageSize (1024 * 4)

#define ExternalFlash_Storage_Address

#define INTERNAL_STORAGE_PAGE_TAG "[Internal Storage]"
#define EXTERNAL_STORAGE_PAGE_TAG "[External Storage]"
#define STORAGE_TAGE "DATA"

typedef uint32_t storage_handle;

typedef enum
{
    Internal_Flash = 0,
    External_Flash,
} Storage_MediumType_List;

typedef struct
{
    uint32_t addr;
    uint32_t map_start_addr;
    uint8_t map_page_num;
    uint32_t info_page_size;
    uint32_t free_block_addr;
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
