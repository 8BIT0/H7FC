#ifndef __STORAGE_H
#define __STORAGE_H

#include "Srv_DataHub.h"
#include "Bsp_Flash.h"

#define OnChipFlash_Storage_StartAddress 
#define OnChipFlash_Stroage_TotalSize

#define OnChilFlash_Storage_PageNum
#define OnChipFlash_Storage_PageSize

#define ExternalFlash_Storage_Address

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

#endif
