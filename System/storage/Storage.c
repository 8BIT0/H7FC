#include "Storage.h"
#include "shell_port.h"

/* internal vriable */
Storage_Monitor_TypeDef Storage_Monitor;
uint8_t page_data_tmp[OnChipFlash_Storage_PageSize] = {0};

/* internal function */
static bool Storage_Build_StorageInfo();
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

static bool Storage_Get_StorageInfo(Storage_MediumType_List type)
{
    switch((uint8_t)type)
    {
        case Internal_Flash:
            if(BspFlash.read(OnChipFlash_Storage_StartAddress, page_data_tmp, OnChipFlash_Storage_InfoPageSize))
            {
                /* check internal storage tag */
                break;
            }
            return false;

        case External_Flash:
            break;

        default:
            return false;
    }

    return false;
}

static bool Storage_Build_StorageInfo(Storage_MediumType_List type)
{
    switch((uint8_t)type)
    {
        case Internal_Flash:
            break;

        case External_Flash:
            break;

        default:
            return false;
    }

    return false;
}

