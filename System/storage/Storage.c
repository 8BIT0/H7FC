#include "Storage.h"
#include "shell_port.h"
#include "util.h"
#include "Srv_OsCommon.h"

#define StorageItem_Size sizeof(Storage_Item_TypeDef)

/* flash io object */
typedef struct
{
    bool (*read)(uint32_t addr_offset, uint8_t *p_data, uint32_t len);
    bool (*write)(uint32_t addr_offset, uint8_t *p_data, uint32_t len);
    bool (*erase)(uint32_t addr_offset, uint32_t len);
} StorageIO_TypeDef;

/* internal vriable */
Storage_Monitor_TypeDef Storage_Monitor;
uint8_t page_data_tmp[OnChipFlash_Storage_TabSize] __attribute__((aligned(4))) = {0};

static bool Storage_OnChipFlash_Read(uint32_t addr_offset, uint8_t *p_data, uint32_t len);
static bool Storage_OnChipFlash_Write(uint32_t addr_offset, uint8_t *p_data, uint32_t len);
static bool Storage_OnChipFlash_Erase(uint32_t addr_offset, uint32_t len);

static bool Storage_Clear_Tab(StorageIO_TypeDef *storage_api, uint32_t addr, uint32_t tab_num);
static bool Storage_Estabish_BootSec_Tab(Storage_MediumType_List type);
static bool Storage_Estabish_SysSec_Tab(Storage_MediumType_List type);
static bool Storage_Estabish_UserSec_Tab(Storage_MediumType_List type);

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
    SrvOsCommon.enter_critical();
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
reformat_internal_flash_info:
            if(Storage_Monitor.InternalFlash_Format_cnt)
            {
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

                    // goto reupdate_internal_flash_info;
                    if(!Storage_Build_StorageInfo(Internal_Flash))
                    {
                        Storage_Monitor.module_init_reg.bit.internal = false;
                        return false;
                    }

                    Storage_Monitor.module_init_reg.bit.internal = true;
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

    SrvOsCommon.exit_critical();
    return Storage_Monitor.init_state;
}

static bool Storage_Format(Storage_MediumType_List type)
{
    StorageIO_TypeDef *StorageIO_API = NULL;
    volatile uint32_t size = 0;
    uint8_t default_data = 0;
    uint32_t read_time = 0;
    volatile uint32_t remain_size = 0;
    uint32_t addr_offset = From_Start_Address;

    switch((uint8_t) type)
    {
        case Internal_Flash:
            StorageIO_API = &InternalFlash_IO;
            size = sizeof(page_data_tmp);
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
            if((remain_size != 0) && (remain_size < size))
                size = remain_size;

            if(!InternalFlash_IO.read(addr_offset, page_data_tmp, size))
                return false;

            for(uint32_t j = 0; j < size; j++)
            {
                if(page_data_tmp[i] != default_data)
                    return false;
            }

            addr_offset += size;
            remain_size -= size;

            if(remain_size == 0)
                return true;
        }
    }

    return false;
}

static bool Storage_Get_StorageInfo(Storage_MediumType_List type)
{
    StorageIO_TypeDef *StorageIO_API = NULL;
    Storage_SectionInfo_TypeDef *p_Info = NULL;
    uint32_t base_addr = 0;
    uint16_t crc = 0;
    uint16_t crc_read = 0;
    char flash_tag[INTERNAL_PAGE_TAG_SIZE + EXTERNAL_PAGE_TAG_SIZE];

    memset(flash_tag, '\0', sizeof(flash_tag));

    switch((uint8_t)type)
    {
        case Internal_Flash:
            StorageIO_API = &InternalFlash_IO;
            memcpy(flash_tag, INTERNAL_STORAGE_PAGE_TAG, INTERNAL_PAGE_TAG_SIZE);
            base_addr = OnChipFlash_Storage_StartAddress;
            p_Info = &Storage_Monitor.internal_info;
            memset(p_Info, 0, sizeof(Storage_SectionInfo_TypeDef));
            break;

        /* still in developping */
        case External_Flash:
        default:
            return false;
    }
    
    if(StorageIO_API->read(From_Start_Address, \
                           page_data_tmp, \
                           sizeof(page_data_tmp)))
    {
        /* check internal storage tag */
        memcpy(p_Info, page_data_tmp, sizeof(Storage_SectionInfo_TypeDef));

        /* check storage tag */
        /* check boot / sys / user  start addr */
        if( (strcmp(p_Info->tag, flash_tag) != 0) || \
            (p_Info->boot_tab_addr == 0) || \
            (p_Info->sys_tab_addr == 0) || \
            (p_Info->user_tab_addr == 0) || \
            (p_Info->boot_tab_addr == p_Info->sys_tab_addr) || \
            (p_Info->boot_tab_addr == p_Info->user_tab_addr) || \
            (p_Info->sys_tab_addr == p_Info->user_tab_addr))
            return false;

        /* get crc from storage baseinfo section check crc value */
        memcpy(&crc_read, &page_data_tmp[OnChipFlash_Storage_InfoPageSize - sizeof(uint16_t)], sizeof(uint16_t));
        crc = Common_CRC16(page_data_tmp, OnChipFlash_Storage_InfoPageSize - sizeof(crc));
        if(crc != crc_read)
            return false;

        return true;
    }

    return false;
}

static bool Storage_Clear_Tab(StorageIO_TypeDef *storage_api, uint32_t addr, uint32_t tab_num)
{
    uint16_t clear_cnt = 1;
    uint32_t clear_size = 0;
    uint32_t clear_remain = 0;
    uint32_t addr_tmp = 0;

    if( (addr == 0) || \
        (tab_num == 0) || \
        (storage_api == NULL) || \
        (storage_api->write == NULL))
        return false;

    for(uint32_t i = 0; i < tab_num; i++)
    {
        if(sizeof(page_data_tmp) < OnChipFlash_Storage_TabSize)
        {
            clear_cnt = OnChipFlash_Storage_TabSize / sizeof(page_data_tmp);
            clear_size = sizeof(page_data_tmp);
            clear_remain = OnChipFlash_Storage_TabSize;
            addr_tmp = addr;
            if(OnChipFlash_Storage_TabSize % sizeof(page_data_tmp))
                clear_cnt += 1;
        }
    
        for(uint8_t c = 0; c < clear_cnt; c++)
        {
            memset(page_data_tmp, 0, clear_size);
            storage_api->write(addr_tmp, page_data_tmp, clear_size);

            clear_remain -= clear_size;
            if(clear_remain <= clear_size)
                clear_size = clear_remain;
            addr_tmp += clear_size;
        }
    }

    return true;
}

static bool Storage_Estabish_BootSec_Tab(Storage_MediumType_List type)
{
    StorageIO_TypeDef *StorageIO_API = NULL;
    Storage_SectionInfo_TypeDef *p_Info = NULL;
    
    switch((uint8_t) type)
    {
        case Internal_Flash:
            StorageIO_API = &InternalFlash_IO;
            if( (!Storage_Monitor.module_init_reg.bit.internal) || \
                (StorageIO_API->erase == NULL) || \
                (StorageIO_API->read  == NULL) || \
                (StorageIO_API->write == NULL))
                return false;

            p_Info = &Storage_Monitor.internal_info;
            break;

        case External_Flash:
            p_Info = &Storage_Monitor.external_info;
        default:
            return false;
    }

    if(p_Info->boot_tab_addr && Storage_Clear_Tab(StorageIO_API, p_Info->boot_tab_addr, p_Info->boot_page_num))
    {
    }

    return false;
}

static bool Storage_Estabish_SysSec_Tab(Storage_MediumType_List type)
{
    StorageIO_TypeDef *StorageIO_API = NULL;
    Storage_SectionInfo_TypeDef *p_Info = NULL;

    switch((uint8_t) type)
    {
        case Internal_Flash:
            StorageIO_API = &InternalFlash_IO;
            if( (!Storage_Monitor.module_init_reg.bit.internal) || \
                (StorageIO_API->erase == NULL) || \
                (StorageIO_API->read  == NULL) || \
                (StorageIO_API->write == NULL))
                return false;

            p_Info = &Storage_Monitor.internal_info;
            break;

        case External_Flash:
            p_Info = &Storage_Monitor.external_info;
        default:
            return false;
    }

    if(p_Info->sys_tab_addr && Storage_Clear_Tab(StorageIO_API, p_Info->sys_tab_addr, p_Info->sys_page_num))
    {
    }
}

static bool Storage_Estabish_UserSec_Tab(Storage_MediumType_List type)
{
    StorageIO_TypeDef *StorageIO_API = NULL;
    Storage_SectionInfo_TypeDef *p_Info = NULL;

    switch((uint8_t) type)
    {
        case Internal_Flash:
            StorageIO_API = &InternalFlash_IO;
            if( (!Storage_Monitor.module_init_reg.bit.internal) || \
                (StorageIO_API->erase == NULL) || \
                (StorageIO_API->read  == NULL) || \
                (StorageIO_API->write == NULL))
                return false;

            p_Info = &Storage_Monitor.internal_info;
            break;

        case External_Flash:
            p_Info = &Storage_Monitor.external_info;
        default:
            return false;
    }

    if(p_Info->user_tab_addr && Storage_Clear_Tab(StorageIO_API, p_Info->user_tab_addr, p_Info->user_page_num))
    {
    }
}

static bool Storage_Build_StorageInfo(Storage_MediumType_List type)
{
    StorageIO_TypeDef *StorageIO_API = NULL;
    uint32_t page_num = 0;
    Storage_SectionInfo_TypeDef Info;
    uint32_t BaseInfo_start_addr = 0;
    uint32_t boot_tab_start_addr = 0;
    uint32_t sys_tab_start_addr = 0;
    uint32_t user_tab_start_addr = 0;
    uint32_t tab_addr_offset = 0;
    uint16_t crc = 0;

    switch((uint8_t)type)
    {
        case Internal_Flash:
            StorageIO_API = &InternalFlash_IO;
            if( (StorageIO_API->erase == NULL) || \
                (StorageIO_API->read  == NULL) || \
                (StorageIO_API->write == NULL))
                return false;

            memset(&Info, 0, sizeof(Storage_SectionInfo_TypeDef));
            memcpy(Info.tag, INTERNAL_STORAGE_PAGE_TAG, strlen(INTERNAL_STORAGE_PAGE_TAG));

            BaseInfo_start_addr = From_Start_Address; 
            page_num = Storage_Max_Capacity / (OnChipFlash_Storage_TabSize / StorageItem_Size);
            if(page_num == 0)
                return false;
            
            Info.boot_tab_addr = BaseInfo_start_addr + OnChipFlash_Storage_InfoPageSize;
            Info.boot_block_size = BootSection_Block_Size;
            Info.boot_page_num = BootTab_Num;
            Info.boot_free_addr = 0;
            Info.boot_para_size = 0;
            Info.boot_para_num = 0;
            tab_addr_offset = (Info.boot_tab_addr + Info.boot_page_num * OnChipFlash_Storage_TabSize);

            Info.sys_tab_addr = tab_addr_offset;
            Info.sys_block_size = page_num * OnChipFlash_Storage_TabSize;
            Info.sys_page_num = page_num;
            Info.sys_free_addr = 0;
            Info.sys_para_size = 0;
            Info.sys_para_num = 0;
            tab_addr_offset += Info.sys_block_size;
                
            Info.user_tab_addr = tab_addr_offset;
            Info.user_block_size = page_num * OnChipFlash_Storage_TabSize;
            Info.user_page_num = page_num;
            Info.user_free_addr = 0;
            Info.user_para_size = 0;
            Info.user_para_num = 0;
            break;

        /* still in developping */
        case External_Flash:
        default:
            return false;
    }

    /* write 0 to info section */
    memset(page_data_tmp, 0, OnChipFlash_Storage_InfoPageSize);
    if(!StorageIO_API->write(BaseInfo_start_addr, page_data_tmp, OnChipFlash_Storage_InfoPageSize))
        return false;

    /* write base info to info section */
    memcpy(page_data_tmp, &Info, sizeof(Info));
    crc = Common_CRC16(page_data_tmp, OnChipFlash_Storage_InfoPageSize - sizeof(crc));
    memcpy(&page_data_tmp[OnChipFlash_Storage_InfoPageSize - sizeof(crc)], &crc, sizeof(crc));

    if(!StorageIO_API->write(BaseInfo_start_addr, page_data_tmp, OnChipFlash_Storage_InfoPageSize))
        return false;
    
    Storage_Estabish_BootSec_Tab(type);
    Storage_Estabish_SysSec_Tab(type);
    Storage_Estabish_UserSec_Tab(type);

    return true;
}

/************************************************** Internal Flash IO API Section ************************************************/
static bool Storage_OnChipFlash_Read(uint32_t addr_offset, uint8_t *p_data, uint32_t len)
{
    uint32_t addr = OnChipFlash_Storage_StartAddress + addr_offset;
    uint32_t read_size = 0;
    uint8_t read_cnt = 1;

    if(len > OnChipFlash_MaxRWSize)
    {
        read_cnt = len / OnChipFlash_MaxRWSize;
        read_size = OnChipFlash_MaxRWSize;
        
        if(len % OnChipFlash_MaxRWSize)
            read_cnt ++;
    }

    for(uint8_t i = 0; i < read_cnt; i++)
    {
        if(!BspFlash.read(addr, p_data, read_size))
            return false;

        addr += read_size;
        len -= read_size;
        p_data += read_size;

        if(len < read_size)
            read_size = len;
    }

    return true;
}

static bool Storage_OnChipFlash_Write(uint32_t addr_offset, uint8_t *p_data, uint32_t len)
{
    uint8_t write_cnt = 0;
    uint32_t write_size = 0;
    uint32_t write_addr = OnChipFlash_Storage_StartAddress + addr_offset;
    
    if(len > OnChipFlash_MaxRWSize)
    {
        write_cnt = len / OnChipFlash_MaxRWSize;
        write_size = OnChipFlash_MaxRWSize;

        if(len % OnChipFlash_MaxRWSize)
            write_cnt ++;
    }
    else
    {
        write_cnt = 1;
        write_size = len;
    }

    for(uint8_t i = 0; i < write_cnt; i++)
    {
        /* erase address first */
        if(!BspFlash.erase(write_addr, write_size))
            return false;
        
        /* after erase write data into address */
        if(!BspFlash.write(write_addr, p_data, write_size))
            return false;

        write_addr += write_size;
        len -= write_size;
        if(len && len <= write_size)
            write_size = len;
    }

    return true;
}

static bool Storage_OnChipFlash_Erase(uint32_t addr_offset, uint32_t len)
{
    uint32_t addr = OnChipFlash_Storage_StartAddress + addr_offset;
    
    /* erase only */
    return BspFlash.erase(addr, len);
}



