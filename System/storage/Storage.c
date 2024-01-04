#include "Storage.h"
#include "shell_port.h"
#include "util.h"

/* flash io object */
typedef struct
{
    bool (*read)(uint32_t addr_offset, uint8_t *p_data, uint32_t len);
    bool (*write)(uint32_t addr_offset, uint8_t *p_data, uint32_t len);
    bool (*erase)(uint32_t addr_offset, uint32_t len);
} StorageIO_TypeDef;

/* internal vriable */
Storage_Monitor_TypeDef Storage_Monitor;
uint8_t page_data_tmp[OnChipFlash_Storage_TabSize] = {0};

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
static bool Storage_Format(Storage_MediumType_List type);

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
        Storage_Monitor.InternalFlash_Format_cnt = Format_Retry_Cnt;
        /* start address check */

        /* flash area size check */

reupdate_internal_flash_info:
        /* read internal flash storage info */
        if(!Storage_Get_StorageInfo(Internal_Flash))
        {
            if(Storage_Monitor.InternalFlash_Format_cnt)
            {
reformat_internal_flash_info:
                Storage_Monitor.InternalFlash_Format_cnt --;

                if(!Storage_Format(Internal_Flash))
                {
                    /* format internal flash storage space */
                    Storage_Monitor.module_init_reg.bit.internal = false;
                    
                    /* format error */
                    goto reformat_internal_flash_info;
                }
                else
                {
                    /* format flash successed */
                    /* build storage tab again */

                    goto reupdate_internal_flash_info;
                }
            }
            else
                Storage_Monitor.module_init_reg.bit.internal = false;
        }
        else
            Storage_Monitor.module_init_reg.bit.internal = true;
    }

    /* still in developping */
    /* external flash init */
    if(enable.bit.external)
    {
        Storage_Monitor.ExternalFlash_Format_cnt = Format_Retry_Cnt;
    }

    Storage_Monitor.init_state = Storage_Monitor.module_init_reg.bit.external | \
                                 Storage_Monitor.module_init_reg.bit.internal;

    return Storage_Monitor.init_state;
}

static bool Storage_Format(Storage_MediumType_List type)
{
    StorageIO_TypeDef *StorageIO_API = NULL;
    uint32_t size = 0;
    uint8_t default_data = 0;
    uint32_t read_time = 0;
    uint32_t remain_size = 0;
    uint32_t addr_offset = From_Start_Address;

    switch((uint8_t) type)
    {
        case Internal_Flash:
            StorageIO_API = &InternalFlash_IO;
            size = OnChipFlash_Storage_TotalSize;
            default_data = OnChipFlash_Storage_DefaultData;

            read_time = OnChipFlash_Storage_TotalSize / sizeof(page_data_tmp);
            if(OnChipFlash_Storage_TotalSize % sizeof(page_data_tmp))
                read_time ++;
            
            remain_size = OnChipFlash_Storage_TotalSize;
            break;
        
        /* still in developping */
        case External_Flash:
        default:
            return false;
    }

    if( InternalFlash_IO.erase && \
        InternalFlash_IO.erase(From_Start_Address, size))
    {
        for(uint32_t i = 0; i < read_time; i++)
        {
            if(remain_size < size)
                size = remain_size;
            
            if(!InternalFlash_IO.read(addr_offset, page_data_tmp, size))
                return false;

            for(uint32_t j = 0; j < size; j++)
            {
                if(page_data_tmp[i] != default_data)
                    return false;
            }

            addr_offset += size;
            remain_size -= sizeof(page_data_tmp);
        }

        return true;
    }

    return false;
}

static bool Storage_Get_StorageInfo(Storage_MediumType_List type)
{
    StorageIO_TypeDef *StorageIO_API = NULL;
    Storage_SectionInfo_TypeDef InfoSec;
    char flash_tag[INTERNAL_PAGE_TAG_SIZE + EXTERNAL_PAGE_TAG_SIZE];
    uint32_t base_addr = 0;

    memset(flash_tag, '\0', sizeof(flash_tag));
    memset(&InfoSec, 0, sizeof(Storage_SectionInfo_TypeDef));

    switch((uint8_t)type)
    {
        case Internal_Flash:
            StorageIO_API = &InternalFlash_IO;
            memcpy(flash_tag, INTERNAL_STORAGE_PAGE_TAG, INTERNAL_PAGE_TAG_SIZE);
            base_addr = OnChipFlash_Storage_StartAddress;
            break;

        /* still in developping */
        case External_Flash:
        default:
            return false;
    }
    
    if(StorageIO_API->read(OnChipFlash_Storage_StartAddress, \
                           page_data_tmp, \
                           OnChipFlash_Storage_InfoPageSize))
    {
        /* check internal storage tag */
        memcpy(&InfoSec, page_data_tmp, sizeof(Storage_SectionInfo_TypeDef));

        /* check storage tag */
        /* check boot / sys / user  start addr */
        if( (strcmp(InfoSec.tag, flash_tag) != 0) || \
            (InfoSec.boot_tab_addr < base_addr) || \
            (InfoSec.sys_tab_addr < base_addr) || \
            (InfoSec.user_tab_addr < base_addr) || \
            (InfoSec.boot_tab_addr == InfoSec.sys_tab_addr) || \
            (InfoSec.boot_tab_addr == InfoSec.user_tab_addr) || \
            (InfoSec.sys_tab_addr == InfoSec.user_tab_addr))
            return false;

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



