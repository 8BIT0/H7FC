#include "Storage.h"
#include "shell_port.h"
#include "util.h"

/* flash io object */
typedef struct
{
    bool (*read)(uint32_t addr, uint8_t *p_data, uint32_t len);
    bool (*write)(uint32_t addr, uint8_t *p_data, uint32_t len);
    bool (*erase)(uint32_t addr, uint32_t len);
} StorageIO_TypeDef;

/* internal vriable */
Storage_Monitor_TypeDef Storage_Monitor;
uint8_t page_data_tmp[OnChipFlash_Storage_PageSize] = {0};
static bool Storage_OnChipFlash_Read(uint32_t addr_offset, uint8_t *p_data, uint32_t len);
static bool Storage_OnChipFlash_Write(uint32_t addr_offset, uint8_t *p_data, uint32_t len);
static bool Storage_OnChipFlash_Erase(uint32_t addr_offset, uint32_t len);

StorageIO_TypeDef InternalFlash_IO = {
    .erase = Storage_OnChipFlash_Erase,
    .read = Storage_OnChipFlash_Read,
    .write = Storage_OnChipFlash_Write,
};

/* internal function */
static bool Storage_Build_StorageInfo(Storage_MediumType_List type);
static bool Storage_Get_StorageInfo(Storage_MediumType_List type);

/* external function */
static bool Storage_Init(Storage_ModuleState_TypeDef enable);

Storage_TypeDef Storage = {
    .init = Storage_Init,
};

static bool Storage_Init(Storage_ModuleState_TypeDef enable)
{
    memset(&Storage_Monitor, 0, sizeof(Storage_Monitor));

    Storage_Monitor.module_enable_reg.val = enable.val;
    Storage_Monitor.module_init_reg.val = 0;

    /* on chip flash init */
    if(enable.bit.internal && BspFlash.init && BspFlash.init())
    {
        /* start address check */

        /* flash area size check */

        /* read internal flash storage info */
        if(!Storage_Get_StorageInfo(Internal_Flash))
            Storage_Monitor.module_init_reg.bit.internal = false;
    }

    /* still in developping */
    /* external flash init */
    if(enable.bit.external)
    {
    }

    Storage_Monitor.init_state = Storage_Monitor.module_init_reg.bit.external | Storage_Monitor.module_init_reg.bit.internal;

    return Storage_Monitor.init_state;
}

static bool Storage_Format(Storage_MediumType_List type)
{
    StorageIO_TypeDef *StorageIO_API = NULL;
    
    switch((uint8_t) type)
    {
        case Internal_Flash:
            StorageIO_API = &InternalFlash_IO;
            break;
        
        /* still in developping */
        case External_Flash:
        default:
            return false;
    }
}

static bool Storage_Get_StorageInfo(Storage_MediumType_List type)
{
    StorageIO_TypeDef *StorageIO_API = NULL;
    
    switch((uint8_t)type)
    {
        case Internal_Flash:
            StorageIO_API = &InternalFlash_IO;
            break;

        /* still in developping */
        case External_Flash:
        default:
            return false;
    }
    
    if(StorageIO_API->read(OnChipFlash_Storage_StartAddress, page_data_tmp, OnChipFlash_Storage_InfoPageSize))
    {
        /* check internal storage tag */

        return true;
    }

    return false;
}

static bool Storage_Build_StorageInfo(Storage_MediumType_List type)
{
    StorageIO_TypeDef *StorageIO_API = NULL;

    switch((uint8_t)type)
    {
        case Internal_Flash:
            StorageIO_API = &InternalFlash_IO;
            break;

        /* still in developping */
        case External_Flash:
        default:
            return false;
    }

    return false;
}

/************************************************** Internal Flash IO API Section ************************************************/
static bool Storage_OnChipFlash_Read(uint32_t addr_offset, uint8_t *p_data, uint32_t len)
{
    uint32_t addr = OnChipFlash_Storage_StartAddress + addr_offset;
    return BspFlash.read(addr, p_data, len);
}

static bool Storage_OnChipFlash_Write(uint32_t addr_offset, uint8_t *p_data, uint32_t len)
{
    uint32_t addr = OnChipFlash_Storage_StartAddress + addr_offset;
    
    /* erase address first */
    if(!BspFlash.erase(addr, len))
        return false;
    
    /* after erase write data into address */
    if(!BspFlash.write(addr, p_data, len))
        return false;

    return true;
}

static bool Storage_OnChipFlash_Erase(uint32_t addr_offset, uint32_t len)
{
    uint32_t addr = OnChipFlash_Storage_StartAddress + addr_offset;
    
    /* erase only */
    return BspFlash.erase(addr, len);
}



