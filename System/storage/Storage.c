/* 
 * Auther: 8_B!T0
 * WARNING: NOT ALL CIRCUMSTANCES BEEN TESTED
 * 
 * Bref: Use for storage parameter for drone
 *       can create search delete parameter section as user want
 */
#include "Storage.h"
#include "shell_port.h"
#include "util.h"
#include "Srv_OsCommon.h"

#define InternalFlash_BootDataSec_Size (4 Kb)
#define InternalFlash_SysDataSec_Size (16 Kb)
#define InternalFlash_UserDataSec_Size (32 Kb)

#define Storage_TabSize Flash_Storage_TabSize
#define Storage_InfoPageSize Flash_Storage_InfoPageSize

#define Item_Capacity_Per_Tab (Storage_TabSize / sizeof(Storage_Item_TypeDef))

/* flash io object */
typedef struct
{
    bool (*read)(uint32_t addr_offset, uint8_t *p_data, uint32_t len);
    bool (*write)(uint32_t addr_offset, uint8_t *p_data, uint32_t len);
    bool (*erase)(uint32_t addr_offset, uint32_t len);
    bool (*erase_all)(void);
} StorageIO_TypeDef;

/* internal vriable */
Storage_Monitor_TypeDef Storage_Monitor;
uint8_t page_data_tmp[Storage_TabSize * 2] __attribute__((aligned(4))) = {0};

static bool Storage_OnChipFlash_Read(uint32_t addr_offset, uint8_t *p_data, uint32_t len);
static bool Storage_OnChipFlash_Write(uint32_t addr_offset, uint8_t *p_data, uint32_t len);
static bool Storage_OnChipFlash_Erase(uint32_t addr_offset, uint32_t len);

static bool Storage_Clear_Tab(StorageIO_TypeDef *storage_api, uint32_t addr, uint32_t tab_num);
static bool Storage_Establish_Tab(Storage_MediumType_List type, Storage_ParaClassType_List class);

static void Storage_Smash_ExternalFlashDev_Ptr(void *bus_cfg_ptr, void *ExtDev_ptr);

static bool Storage_ExtFlash_Read(uint32_t addr_offset, uint8_t *p_data, uint32_t len);
static bool Storage_ExtFlash_Write(uint32_t addr_offset, uint8_t *p_data, uint32_t len);
static bool Storage_ExtFlash_Erase(uint32_t addr_offset, uint32_t len);
static bool Storage_ExtFlash_EraseAll(void);

static Storage_BaseSecInfo_TypeDef* Storage_Get_SecInfo(Storage_FlashInfo_TypeDef *info, Storage_ParaClassType_List class);

StorageIO_TypeDef InternalFlash_IO = {
    .erase = Storage_OnChipFlash_Erase,
    .erase_all = NULL,
    .read = Storage_OnChipFlash_Read,
    .write = Storage_OnChipFlash_Write,
};

StorageIO_TypeDef ExternalFlash_IO = {
    .erase = Storage_ExtFlash_Erase,
    .erase_all = Storage_ExtFlash_EraseAll,
    .read = Storage_ExtFlash_Read,
    .write = Storage_ExtFlash_Write,
};

/* internal function */
static bool Storage_External_Chip_W25Qxx_SelectPin_Ctl(bool state);
static uint16_t Storage_External_Chip_W25Qxx_BusTx(uint8_t *p_data, uint16_t len, uint32_t time_out);
static uint16_t Storage_External_Chip_W25Qxx_BusRx(uint8_t *p_data, uint16_t len, uint32_t time_out);
static uint16_t Storage_External_Chip_W25Qxx_BusTrans(uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t time_out);
static bool Storage_Build_StorageInfo(Storage_MediumType_List type);
static bool Storage_Get_StorageInfo(Storage_MediumType_List type);
static bool Storage_Format(Storage_MediumType_List type);
static Storage_ErrorCode_List Storage_CreateItem(Storage_MediumType_List type, Storage_ParaClassType_List class, const char *name, uint8_t *p_data, uint32_t size);
 
/* external function */
static bool Storage_Init(Storage_ModuleState_TypeDef enable, Storage_ExtFLashDevObj_TypeDef *ExtDev);
static storage_handle Storage_Search(Storage_MediumType_List medium, Storage_ParaClassType_List class, const char *name);

Storage_TypeDef Storage = {
    .init = Storage_Init,
};

static bool Storage_Init(Storage_ModuleState_TypeDef enable, Storage_ExtFLashDevObj_TypeDef *ExtDev)
{
    void *ext_flash_bus_cfg = NULL;

    memset(&Storage_Monitor, 0, sizeof(Storage_Monitor));

    Storage_Monitor.module_enable_reg.val = enable.val;
    Storage_Monitor.module_init_reg.val = 0;

    /* on chip flash init */
    if (enable.bit.internal && BspFlash.init && BspFlash.init())
    {
        Storage_Monitor.InternalFlash_Format_cnt = Format_Retry_Cnt;
        /* start address check */

        /* flash area size check */

reupdate_internal_flash_info:
        /* read internal flash storage info */
        if (!Storage_Get_StorageInfo(Internal_Flash))
        {
            Storage_Monitor.internal_info.base_addr = OnChipFlash_Storage_StartAddress;
reformat_internal_flash_info:
            if (Storage_Monitor.InternalFlash_Format_cnt)
            {
                Storage_Monitor.InternalFlash_Format_cnt --;

                if (!Storage_Format(Internal_Flash))
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
                    if (Storage_Build_StorageInfo(Internal_Flash))
                    {
                        Storage_Monitor.module_init_reg.bit.internal = true;
                        goto reupdate_internal_flash_info;
                    }
                    else
                        Storage_Monitor.module_init_reg.bit.internal = false;
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
    if (enable.bit.external && \
        ExtDev && \
        (ExtDev->chip_type != Storage_Chip_None))
    {
        if (ExtDev->bus_type == Storage_ChipBus_Spi)
        {
            Storage_Monitor.ExtDev_ptr = NULL;
            ext_flash_bus_cfg = SrvOsCommon.malloc(sizeof(BspSPI_NorModeConfig_TypeDef));

            if (ext_flash_bus_cfg)
            {
                memset(ext_flash_bus_cfg, 0, sizeof(BspSPI_NorModeConfig_TypeDef));

                To_NormalSPI_ObjPtr(ext_flash_bus_cfg)->BaudRatePrescaler = ExtFlash_Bus_Clock_Div;
                To_NormalSPI_ObjPtr(ext_flash_bus_cfg)->CLKPhase = ExtFlash_Bus_CLKPhase;
                To_NormalSPI_ObjPtr(ext_flash_bus_cfg)->CLKPolarity = ExtFlash_Bus_CLKPolarity;
                To_NormalSPI_ObjPtr(ext_flash_bus_cfg)->Instance = ExtFLash_Bus_Instance;
                To_NormalSPI_ObjPtr(ext_flash_bus_cfg)->Pin = ExtFlash_Bus_Pin;

                /* bus init & cs pin init */
                if (ExtFlash_Bus_Api.init(To_NormalSPI_Obj(ext_flash_bus_cfg), ExtFLash_Bus_Instance) && \
                    BspGPIO.out_init(ExtFlash_CS_Pin))
                {
                    Storage_Monitor.ExtBusCfg_Ptr = ext_flash_bus_cfg;

                    if (ExtDev->chip_type == Storage_ChipType_W25Qxx)
                    {
                        /* set get time callback */
                        To_DevW25Qxx_OBJ(ExtDev->dev_obj)->systick = SrvOsCommon.get_os_ms;

                        /* set bus control callback */
                        To_DevW25Qxx_OBJ(ExtDev->dev_obj)->cs_ctl = Storage_External_Chip_W25Qxx_SelectPin_Ctl;
                        To_DevW25Qxx_OBJ(ExtDev->dev_obj)->bus_tx = Storage_External_Chip_W25Qxx_BusTx;
                        To_DevW25Qxx_OBJ(ExtDev->dev_obj)->bus_rx = Storage_External_Chip_W25Qxx_BusRx;
                        To_DevW25Qxx_OBJ(ExtDev->dev_obj)->bus_trans = Storage_External_Chip_W25Qxx_BusTrans;

                        if (To_DevW25Qxx_API(ExtDev->dev_api)->init(To_DevW25Qxx_OBJ(ExtDev->dev_obj)) == DevW25Qxx_Ok)
                        {
                            Storage_Monitor.ExtDev_ptr = ExtDev;
                            ExtDev->sector_num  = To_DevW25Qxx_API(ExtDev->dev_api)->info(To_DevW25Qxx_OBJ(ExtDev->dev_obj)).subsector_num;
                            ExtDev->sector_size = To_DevW25Qxx_API(ExtDev->dev_api)->info(To_DevW25Qxx_OBJ(ExtDev->dev_obj)).subsector_size;
                            ExtDev->total_size  = To_DevW25Qxx_API(ExtDev->dev_api)->info(To_DevW25Qxx_OBJ(ExtDev->dev_obj)).flash_size;
                            ExtDev->page_num    = To_DevW25Qxx_API(ExtDev->dev_api)->info(To_DevW25Qxx_OBJ(ExtDev->dev_obj)).page_num;
                            ExtDev->page_size   = To_DevW25Qxx_API(ExtDev->dev_api)->info(To_DevW25Qxx_OBJ(ExtDev->dev_obj)).page_size;

                            /* set external flash device read write base address */
                            Storage_Monitor.external_info.base_addr = ExtFlash_Start_Addr;
                            Storage_Monitor.ExternalFlash_Format_cnt = Format_Retry_Cnt;

reupdate_external_flash_info:
                            /* get storage info */
                            if (!Storage_Get_StorageInfo(External_Flash))
                            {
reformat_external_flash_info:
                                if (Storage_Monitor.ExternalFlash_Format_cnt)
                                {
                                    /* format storage device */
                                    if (!Storage_Format(External_Flash))
                                    {
                                        Storage_Monitor.ExternalFlash_Format_cnt --;
                                        Storage_Monitor.external_info.base_addr = ExtFlash_Start_Addr;
                                        if (Storage_Monitor.ExternalFlash_Format_cnt)
                                            goto reformat_external_flash_info;
                                    }
                                    else
                                    {
                                        /* external flash module format successed */
                                        /* build storage tab */
                                        if (Storage_Build_StorageInfo(External_Flash))
                                        {
                                            Storage_Monitor.module_init_reg.bit.external = true;
                                            /* after tab builded read storage info again */
                                            goto reupdate_external_flash_info;
                                        }
                                        else
                                            Storage_Monitor.module_init_reg.bit.internal = false;
                                    }
                                }
                                else
                                {
                                    /* reformat count is 0 */
                                    /* external flash module format error */
                                    Storage_Monitor.module_init_reg.bit.external = false;
                                    Storage_Smash_ExternalFlashDev_Ptr(ext_flash_bus_cfg, ExtDev);
                                }
                            }
                            else
                                Storage_Monitor.module_init_reg.bit.external = true;
                        }
                        else
                            Storage_Smash_ExternalFlashDev_Ptr(ext_flash_bus_cfg, ExtDev);
                    }
                    else
                        Storage_Smash_ExternalFlashDev_Ptr(ext_flash_bus_cfg, ExtDev);
                }
                else
                    Storage_Smash_ExternalFlashDev_Ptr(ext_flash_bus_cfg, ExtDev);
            }
            else
                Storage_Smash_ExternalFlashDev_Ptr(ext_flash_bus_cfg, ExtDev);
        }
        else
        {
            SrvOsCommon.free(ExtDev);
            Storage_Monitor.module_init_reg.bit.external = false;
        }
    }
    else
    {
        if (ExtDev)
            SrvOsCommon.free(ExtDev);

        Storage_Monitor.module_init_reg.bit.external = false;
    }

    Storage_Monitor.init_state = Storage_Monitor.module_init_reg.bit.external | \
                                 Storage_Monitor.module_init_reg.bit.internal;

    return Storage_Monitor.init_state;
}

static void Storage_Smash_ExternalFlashDev_Ptr(void *bus_cfg_ptr, void *ExtDev_ptr)
{
    if((bus_cfg_ptr == NULL) || (ExtDev_ptr == NULL))
        return;

    SrvOsCommon.free(bus_cfg_ptr);
    Storage_Monitor.module_init_reg.bit.external = false;
    SrvOsCommon.free(ExtDev_ptr);
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
            size = Storage_TabSize;
            default_data = OnChipFlash_Storage_DefaultData;

            read_time = OnChipFlash_Storage_TotalSize / Storage_TabSize;
            if(OnChipFlash_Storage_TotalSize % Storage_TabSize)
                read_time ++;
            
            remain_size = OnChipFlash_Storage_TotalSize;
            break;
        
        /* still in developping */
        case External_Flash:
            StorageIO_API = &ExternalFlash_IO;
            size = Storage_TabSize;
            default_data = ExtFlash_Storage_DefaultData;

            read_time = ExtFlash_Storage_TotalSize / Storage_TabSize;
            if(ExtFlash_Storage_TotalSize % Storage_TabSize)
                read_time ++;

            remain_size = ExtFlash_Storage_TotalSize;
            break;

        default:
            return false;
    }

    if (StorageIO_API->erase)
    {
        for(uint32_t i = 0; i < read_time; i++)
        {
            if((remain_size != 0) && (remain_size < size))
                size = remain_size;

            if (!StorageIO_API->erase(addr_offset, size) || 
                !StorageIO_API->read(addr_offset, page_data_tmp, size))
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

/* still in developping */
static bool Storage_Check_Tab(StorageIO_TypeDef *storage_api, Storage_BaseSecInfo_TypeDef *sec_info)
{
    uint32_t free_i = 0;
    uint32_t tab_addr = 0;
    uint32_t store_param_found = 0;
    uint32_t store_param_size = 0;
    uint32_t free_slot_addr = 0;
    uint32_t sec_start_addr = 0;
    uint32_t sec_end_addr = 0;
    uint16_t crc16 = 0;
    uint8_t *crc_buf = NULL;
    uint16_t crc_len = 0;
    Storage_FreeSlot_TypeDef *FreeSlot_Info = NULL;
    Storage_Item_TypeDef *p_ItemList = NULL;

    if (storage_api && storage_api->read && sec_info)
    {
        /* free address & slot check */
        free_slot_addr = sec_info->free_slot_addr;
        sec_start_addr = sec_info->data_sec_addr;
        sec_end_addr = sec_start_addr + sec_info->data_sec_size;

        for(free_i = 0; ;)
        {
            /* check boot section free slot */
            if ((free_slot_addr == 0) || \
                (free_slot_addr < sec_start_addr) || \
                (free_slot_addr > sec_end_addr) || \
                !storage_api->read(free_slot_addr, page_data_tmp, Storage_TabSize))
                break;

            FreeSlot_Info = (Storage_FreeSlot_TypeDef *)page_data_tmp;

            if ((FreeSlot_Info->head_tag != STORAGE_SLOT_HEAD_TAG) || \
                (FreeSlot_Info->end_tag != STORAGE_SLOT_END_TAG))
                return false;

            free_i ++;
            if (FreeSlot_Info->nxt_addr == 0)
                break;

            free_slot_addr = FreeSlot_Info->nxt_addr;
        }

        if ((sec_info->free_slot_num != 0) && \
            (free_i != sec_info->free_slot_num))
            return false;
        
        if (sec_info->para_num)
        {
            tab_addr = sec_info->tab_addr;

            for (uint16_t tab_i = 0; tab_i < sec_info->page_num; tab_i ++)
            {
                if (!storage_api->read(tab_addr, page_data_tmp, sec_info->tab_size))
                    return false;
            
                p_ItemList = page_data_tmp;
                for(uint16_t item_i = 0; item_i < Item_Capacity_Per_Tab; item_i ++)
                {
                    if ((p_ItemList[item_i].head_tag == STORAGE_ITEM_HEAD_TAG) && \
                        (p_ItemList[item_i].end_tag == STORAGE_ITEM_END_TAG))
                    {
                        /* check item slot crc */
                        /*
                         *  typedef struct
                         *  {
                         *      uint8_t head_tag;
                         *      uint8_t class;
                         *      uint8_t name[STORAGE_NAME_LEN];
                         *      uint32_t data_addr;
                         *      uint16_t len;
                         *      uint16_t crc16;
                         *      uint8_t end_tag;
                         *  } Storage_Item_TypeDef;
                         *  
                         * comput crc from class to len
                         */
                        crc_buf = &p_ItemList[item_i] + sizeof(p_ItemList[item_i].head_tag);
                        crc_len = sizeof(Storage_Item_TypeDef);
                        crc_len -= sizeof(p_ItemList[item_i].head_tag);
                        crc_len -= sizeof(p_ItemList[item_i].end_tag);
                        crc_len -= sizeof(p_ItemList[item_i].crc16);
                        
                        crc16 = Common_CRC16(crc_buf, crc_len);
                        if (crc16 != p_ItemList[item_i].crc16)
                            return false;

                        store_param_found ++;
                    }
                }

                tab_addr += sec_info->tab_size;
            }

            if ((store_param_found != sec_info->para_num) || \
                (store_param_size != sec_info->para_size))
                return false;
        }

        return true;
    }

    return false;
}

static bool Storage_Get_StorageInfo(Storage_MediumType_List type)
{
    StorageIO_TypeDef *StorageIO_API = NULL;
    Storage_FlashInfo_TypeDef *p_Info = NULL;
    uint16_t i = 0;
    uint16_t crc = 0;
    uint16_t crc_read = 0;
    uint32_t info_addr = 0;
    char flash_tag[INTERNAL_PAGE_TAG_SIZE + EXTERNAL_PAGE_TAG_SIZE];

    memset(flash_tag, '\0', sizeof(flash_tag));

    switch((uint8_t)type)
    {
        case Internal_Flash:
            StorageIO_API = &InternalFlash_IO;
            memcpy(flash_tag, INTERNAL_STORAGE_PAGE_TAG, INTERNAL_PAGE_TAG_SIZE);
            p_Info = &Storage_Monitor.internal_info;
            info_addr = From_Start_Address;
            break;

        /* still in developping */
        case External_Flash:
            StorageIO_API = &ExternalFlash_IO;
            memcpy(flash_tag, EXTERNAL_STORAGE_PAGE_TAG, EXTERNAL_PAGE_TAG_SIZE);
            p_Info = &Storage_Monitor.external_info;
            info_addr = p_Info->base_addr;
            break;

        default:
            return false;
    }
    
    memset(p_Info, 0, sizeof(Storage_FlashInfo_TypeDef));
    if (StorageIO_API->read(info_addr, page_data_tmp, Storage_TabSize))
    {
        /* check internal storage tag */
        memcpy(p_Info, page_data_tmp, sizeof(Storage_FlashInfo_TypeDef));
        p_Info->base_addr = info_addr;
        
        /* check storage tag */
        /* check boot / sys / user  start addr */
        if ((strcmp(p_Info->tag, flash_tag) != 0) || \
            (p_Info->base_addr != info_addr) || \
            (p_Info->boot_sec.tab_addr == 0) || \
            (p_Info->sys_sec.tab_addr == 0) || \
            (p_Info->user_sec.tab_addr == 0) || \
            (p_Info->boot_sec.tab_addr == p_Info->sys_sec.tab_addr) || \
            (p_Info->boot_sec.tab_addr == p_Info->user_sec.tab_addr) || \
            (p_Info->sys_sec.tab_addr == p_Info->user_sec.tab_addr))
            return false;

        /* get crc from storage baseinfo section check crc value */
        memcpy(&crc_read, &page_data_tmp[OnChipFlash_Storage_InfoPageSize - sizeof(uint16_t)], sizeof(uint16_t));
        crc = Common_CRC16(page_data_tmp, OnChipFlash_Storage_InfoPageSize - sizeof(crc));
        if(crc != crc_read)
            return false;

        memset(page_data_tmp, 0, Storage_TabSize);
        /* check  boot  section tab & free slot info & stored item */
        /* check system section tab & free slot info & stored item */
        /* check  user  section tab & free slot info & stored item */
        if (Storage_Check_Tab(StorageIO_API, &p_Info->boot_sec) && \
            Storage_Check_Tab(StorageIO_API, &p_Info->sys_sec) && \
            Storage_Check_Tab(StorageIO_API, &p_Info->user_sec))
            return true;

        return true;
    }

    return false;
}

static bool Storage_Clear_Tab(StorageIO_TypeDef *storage_api, uint32_t addr, uint32_t tab_num)
{
    uint32_t addr_tmp = 0;

    if ((addr == 0) || \
        (tab_num == 0) || \
        (storage_api == NULL) || \
        (storage_api->write == NULL))
        return false;

    memset(page_data_tmp, 0, Storage_TabSize);
    addr_tmp = addr;
    
    for(uint32_t i = 0; i < tab_num; i++)
    {    
        if (!storage_api->write(addr_tmp, page_data_tmp, Storage_TabSize))
            return false;

        addr_tmp += Storage_TabSize;
    }

    return true;
}

static bool Storage_DeleteItem(Storage_MediumType_List type, Storage_ParaClassType_List class, const char *name, uint32_t size)
{
    Storage_BaseSecInfo_TypeDef *p_SecInfo = NULL;
    Storage_Item_TypeDef *item_list = NULL;
    StorageIO_TypeDef *StorageIO_API = NULL;
    
    if( !Storage_Monitor.init_state || \
        (name == NULL) || \
        (strlen(name) == 0) || \
        (strlen(name) >= STORAGE_NAME_LEN) || \
        (size == 0))
        return false;
    


    return false;
}

static Storage_ErrorCode_List Storage_CreateItem(Storage_MediumType_List type, Storage_ParaClassType_List class, const char *name, uint8_t *p_data, uint32_t size)
{
    uint8_t *crc_buf = NULL;
    uint16_t crc_len = 0;
    uint32_t storage_data_size = 0;
    uint32_t stored_size = 0;
    uint32_t store_addr = 0;
    uint32_t unstored_size = 0;
    uint32_t storage_tab_addr = 0;
    uint32_t cur_freeslot_addr = 0;
    uint32_t nxt_freeslot_addr = 0;
    uint32_t slot_useful_size = 0;
    uint32_t free_space_remianing = 0;
    uint8_t item_index = 0;
    StorageIO_TypeDef *StorageIO_API = NULL;
    Storage_FlashInfo_TypeDef *p_Flash = NULL;
    Storage_BaseSecInfo_TypeDef *p_Sec = NULL;
    Storage_Item_TypeDef *tab_item = NULL;
    Storage_Item_TypeDef crt_item_slot;
    Storage_FreeSlot_TypeDef FreeSlot;
    Storage_FreeSlot_TypeDef New_FreeSlot;
    Storage_DataSlot_TypeDef DataSlot;

    memset(&crt_item_slot, 0, sizeof(crt_item_slot));
    memset(&DataSlot, 0, sizeof(Storage_DataSlot_TypeDef));
    memset(&FreeSlot, 0, sizeof(Storage_FreeSlot_TypeDef));
    memset(&New_FreeSlot, 0, sizeof(Storage_FreeSlot_TypeDef));

    if ((name == NULL) || (p_data == NULL) || (size == 0))
        return Storage_Param_Error;

    switch((uint8_t)type)
    {
        case Internal_Flash:
            if (Storage_Monitor.module_enable_reg.bit.internal && \
                Storage_Monitor.module_init_reg.bit.internal)
            {
                StorageIO_API = &InternalFlash_IO;
                p_Flash = &Storage_Monitor.internal_info;
                break;
            }
            return Storage_ModuleInit_Error;

        case External_Flash :
            if (Storage_Monitor.module_enable_reg.bit.external && \
                Storage_Monitor.module_init_reg.bit.external)
            {
                StorageIO_API = &ExternalFlash_IO;
                p_Flash = &Storage_Monitor.external_info;
                break;
            }
            return Storage_ModuleInit_Error;

        default:
            return Storage_ModuleType_Error;
    }

    p_Sec = Storage_Get_SecInfo(p_Flash, class);
    if (p_Sec == NULL)
        return Storage_Class_Error;

    if ((StorageIO_API->erase == NULL) || \
        (StorageIO_API->read == NULL) || \
        (StorageIO_API->write == NULL))
        return Storage_ModuleAPI_Error;

    if (p_Sec->free_slot_addr && \
        StorageIO_API->read(p_Sec->free_slot_addr, &FreeSlot, sizeof(Storage_FreeSlot_TypeDef)) && \
        (FreeSlot.head_tag == STORAGE_SLOT_HEAD_TAG) && \
        (FreeSlot.end_tag == STORAGE_SLOT_END_TAG) && \
        (strlen(name) <= STORAGE_NAME_LEN) && \
        (FreeSlot.total_size >= size))
    {
        cur_freeslot_addr = p_Sec->free_slot_addr;
        free_space_remianing = FreeSlot.total_size;

        /* step 1: update info section first */
        p_Sec->para_num ++;
        p_Sec->para_size += size;
        
        storage_tab_addr = p_Sec->tab_addr;
        for(uint16_t tab_i = 0; tab_i < p_Sec->page_num; tab_i ++)
        {
            /* step 2: update tab */
            if (!StorageIO_API->read(storage_tab_addr, page_data_tmp, p_Sec->tab_size))
                return Storage_Read_Error;

            tab_item = page_data_tmp;
            for (uint16_t item_i = 0; item_i < Item_Capacity_Per_Tab; item_i ++)
            {
                if ((tab_item[item_i].head_tag != STORAGE_ITEM_HEAD_TAG) && \
                    (tab_item[item_i].end_tag != STORAGE_ITEM_END_TAG))
                {
                    item_index = item_i;

                    /* found empty item slot */
                    crt_item_slot = tab_item[item_i];

                    /* set item slot info */
                    crt_item_slot.class = class;
                    memset(crt_item_slot.name, '\0', STORAGE_NAME_LEN);
                    memcpy(crt_item_slot.name, name, strlen(name));
                    crt_item_slot.len = size;

                    /* set free slot address as current data address */
                    crt_item_slot.data_addr = p_Sec->free_slot_addr;
                    crt_item_slot.head_tag = STORAGE_ITEM_HEAD_TAG;
                    crt_item_slot.end_tag = STORAGE_ITEM_END_TAG;

                    /* comput crc */
                    crc_buf = &crt_item_slot + sizeof(crt_item_slot.head_tag);
                    crc_len = sizeof(Storage_Item_TypeDef);
                    crc_len -= sizeof(crt_item_slot.head_tag);
                    crc_len -= sizeof(crt_item_slot.end_tag);
                    crc_len -= sizeof(crt_item_slot.crc16);

                    crt_item_slot.crc16 = Common_CRC16(crc_buf, crc_len);
                    store_addr = crt_item_slot.data_addr;
                    break; 
                }
            }

            storage_tab_addr += p_Sec->tab_size;
        }

        if (store_addr == 0)
            return Storage_DataAddr_Update_Error;

        storage_data_size = sizeof(Storage_DataSlot_TypeDef);
        storage_data_size += size;
        if (size % STORAGE_DATA_ALIGN)
            /* get align byte size */
            /* noticed: write 0 on align space */
            storage_data_size += size % STORAGE_DATA_ALIGN;

        /* noticed: DataSlot.total_data_size - DataSlot.align_size is storaged data size */
        DataSlot.total_data_size = storage_data_size;
        unstored_size = DataSlot.total_data_size;
        
        if (FreeSlot.total_size >= (sizeof(Storage_DataSlot_TypeDef) + size))
        {
            while(true)
            {
                /* step 3: comput storage data size and set data slot */
                DataSlot.head_tag = STORAGE_SLOT_HEAD_TAG;
                DataSlot.end_tag = STORAGE_SLOT_END_TAG;
                memset(DataSlot.name, '\0', STORAGE_NAME_LEN);
                memcpy(DataSlot.name, name, strlen(name));
                
                Storage_Assert(FreeSlot.cur_slot_size <= sizeof(Storage_DataSlot_TypeDef));

                slot_useful_size = FreeSlot.cur_slot_size - sizeof(Storage_DataSlot_TypeDef);
                /* current have space for new data need to be storage */
                if (slot_useful_size <= storage_data_size)
                {
                    DataSlot.cur_slot_size = slot_useful_size;
                    stored_size += DataSlot.cur_slot_size;
                    unstored_size -= stored_size;
                    DataSlot.align_size = 0;
                    free_space_remianing -= FreeSlot.cur_slot_size;
                    
                    /* current free slot full fill can not split any space for next free slot`s start */
                    DataSlot.nxt_addr = FreeSlot.nxt_addr;

                    /* in light of current free slot not enough for storage data, 
                     * then find next free slot used for storage data remaining */
                    if (!StorageIO_API->read(FreeSlot.nxt_addr, &FreeSlot, sizeof(Storage_FreeSlot_TypeDef)))
                        return Storage_FreeSlot_Get_Error;
                }
                else
                {
                    /* seperate data slot and new free slot from current free slot */
                    stored_size += unstored_size;
                    DataSlot.cur_slot_size = unstored_size;
                    DataSlot.align_size = size % STORAGE_DATA_ALIGN;
                    DataSlot.nxt_addr = 0;
                    free_space_remianing -= unstored_size + sizeof(Storage_DataSlot_TypeDef);

                    New_FreeSlot.total_size = free_space_remianing;
                    // New_FreeSlot.nxt_addr = cur_freeslot_addr + ;
                    New_FreeSlot.cur_slot_size -= DataSlot.cur_slot_size + sizeof(Storage_DataSlot_TypeDef);
                    New_FreeSlot.nxt_addr += sizeof(Storage_DataSlot_TypeDef);
                }

                /* comput current slot crc */
                memcpy(page_data_tmp, 0, DataSlot.cur_slot_size);

                /* write to the data section */
                /* storage target data */
                // if (!StorageIO_API->write(store_addr, , ))
                //     return ;

                if (DataSlot.nxt_addr == 0)
                {
                    Storage_Assert(DataSlot.total_data_size > stored_size);
                    if (DataSlot.total_data_size == stored_size)
                        break;

                    return Storage_No_Enough_Space;
                }
                else
                {
                    /* after target data segment stored, shift target data pointer to unstored pos
                     * and update next segment data store address */
                    p_data += slot_useful_size;
                    store_addr = DataSlot.nxt_addr;
                }
            }

            /* get tab */
            if (!StorageIO_API->read(storage_tab_addr, page_data_tmp, p_Sec->tab_addr))
                return Storage_Read_Error;

            tab_item = page_data_tmp;
            memcpy(&tab_item[item_index], &crt_item_slot, sizeof(Storage_Item_TypeDef));

            /* write back item slot list to tab */
            if (!StorageIO_API->write(storage_tab_addr, page_data_tmp, p_Sec->tab_addr))
                return Storage_Write_Error;
        }
        else
            /* don`t have enough space for target data */
            return Storage_No_Enough_Space;

        /* step 4: update free slot */
        FreeSlot.total_size -= storage_data_size;
        New_FreeSlot = FreeSlot;
        New_FreeSlot.total_size -= size;
    }

    return Storage_Error_None;
}

static bool Storage_Establish_Tab(Storage_MediumType_List type, Storage_ParaClassType_List class)
{
    StorageIO_TypeDef *StorageIO_API = NULL;
    Storage_FlashInfo_TypeDef *p_Flash = NULL;
    Storage_BaseSecInfo_TypeDef *p_SecInfo = NULL;
    uint16_t clear_cnt = 0;
    uint32_t clear_byte = 0;
    uint32_t clear_remain = 0;
    uint32_t addr_tmp = 0;
    Storage_FreeSlot_TypeDef free_slot;
    uint16_t crc = 0;

    switch((uint8_t) type)
    {
        case Internal_Flash:
            StorageIO_API = &InternalFlash_IO;
            p_Flash = &Storage_Monitor.internal_info;
            break;

        case External_Flash:
            StorageIO_API = &ExternalFlash_IO;
            p_Flash = &Storage_Monitor.external_info;
            break;

        default:
            return false;
    }
    
    if ((StorageIO_API->erase == NULL) || \
        (StorageIO_API->read  == NULL) || \
        (StorageIO_API->write == NULL))
        return false;

    p_SecInfo = Storage_Get_SecInfo(p_Flash, class);
    if (p_SecInfo == NULL)
        return false;

    if (p_SecInfo->tab_addr && Storage_Clear_Tab(StorageIO_API, p_SecInfo->tab_addr, p_SecInfo->page_num))
    {
        /* clear boot data section */
        if (p_SecInfo->data_sec_addr == 0)
            return false;

        /* write 0 to data section */
        memset(page_data_tmp, 0, Storage_TabSize);
        clear_cnt = p_SecInfo->data_sec_size / Storage_TabSize;
        clear_remain = p_SecInfo->data_sec_size;
        addr_tmp = p_SecInfo->data_sec_addr;
        clear_byte = Storage_TabSize;
        if(p_SecInfo->data_sec_size % Storage_TabSize)
            clear_cnt ++;
 
        for(uint16_t i = 0; i < clear_cnt; i++)
        {
            if (!StorageIO_API->write(addr_tmp, page_data_tmp, clear_byte))
                return false;

            addr_tmp += Storage_TabSize;
            clear_remain -= clear_byte;
            if(clear_remain && clear_remain <= clear_byte)
                clear_byte = clear_remain;
        }

        /* write free slot info */
        memset(&free_slot, 0, sizeof(free_slot));
        free_slot.head_tag = STORAGE_SLOT_HEAD_TAG;
        free_slot.total_size = p_SecInfo->data_sec_size - sizeof(free_slot);
        free_slot.cur_slot_size = p_SecInfo->data_sec_size - sizeof(free_slot);
        free_slot.nxt_addr = 0;
        /* if current free slot get enought space for data then set ender tag as 0xFE1001FE */
        /* or else set ender tag as next slot address such like 0x80exxxx. */
        /* until all data was saved into multiple flash segment completely */
        /* set ender tag as 0xFE1001FE */
        free_slot.end_tag = STORAGE_SLOT_END_TAG;
        
        memcpy(page_data_tmp, &free_slot, sizeof(free_slot));
        if (!StorageIO_API->write(p_SecInfo->data_sec_addr, page_data_tmp, Storage_TabSize))
            return false;

        /* update info section */
        p_SecInfo->free_slot_addr = p_SecInfo->data_sec_addr;
        p_SecInfo->free_slot_num = 1;
        p_SecInfo->para_num = 0;
        p_SecInfo->para_size = 0;

        /* read out whole section info data from storage info section */
        if (!StorageIO_API->read(From_Start_Address, page_data_tmp, Storage_TabSize))
            return false;

        memset(page_data_tmp, 0, Storage_InfoPageSize);
        memcpy(page_data_tmp, p_Flash, sizeof(Storage_FlashInfo_TypeDef));

        /* comput crc */
        crc = Common_CRC16(page_data_tmp, Storage_InfoPageSize - sizeof(crc));
        memcpy(&page_data_tmp[Storage_InfoPageSize - sizeof(crc)], &crc, sizeof(crc));

        /* erase sector first then write into the target sector */
        if (!StorageIO_API->write(From_Start_Address, page_data_tmp, Storage_TabSize))
            return false;

        return true;
    }

    return false;
}

static bool Storage_Build_StorageInfo(Storage_MediumType_List type)
{
    StorageIO_TypeDef *StorageIO_API = NULL;
    uint32_t page_num = 0;
    Storage_FlashInfo_TypeDef Info;
    Storage_FlashInfo_TypeDef Info_Rx;
    uint32_t addr_offset = 0;
    uint32_t BaseInfo_start_addr = 0;
    uint32_t boot_tab_start_addr = 0;
    uint32_t sys_tab_start_addr = 0;
    uint32_t user_tab_start_addr = 0;
    uint32_t tab_addr_offset = 0;
    uint16_t crc = 0;
    uint32_t data_sec_addr = 0;
    uint32_t remain_data_sec_size = 0;
    uint32_t data_sec_size = 0;
    uint32_t info_page_size = 0;
    uint32_t sector_size = 0;

    memset(&Info, 0, sizeof(Storage_FlashInfo_TypeDef));
    memset(&Info_Rx, 0, sizeof(Storage_FlashInfo_TypeDef));
    
    switch((uint8_t)type)
    {
        case Internal_Flash:
            info_page_size = OnChipFlash_Storage_InfoPageSize;
            StorageIO_API = &InternalFlash_IO;
            memcpy(Info.tag, INTERNAL_STORAGE_PAGE_TAG, INTERNAL_PAGE_TAG_SIZE);

            if ((StorageIO_API->erase == NULL) || \
                (StorageIO_API->read  == NULL) || \
                (StorageIO_API->write == NULL))
                return false;

            Info.total_size = OnChipFlash_Storage_TotalSize;

            BaseInfo_start_addr = From_Start_Address;
            page_num = Storage_OnChip_Max_Capacity / (OnChipFlash_Storage_TabSize / StorageItem_Size);
            if (page_num == 0)
                return false;
            
            Info.boot_sec.tab_addr = BaseInfo_start_addr + info_page_size;
            Info.boot_sec.tab_size = BootSection_Block_Size * BootTab_Num;
            Info.boot_sec.page_num = BootTab_Num;
            Info.boot_sec.data_sec_size = InternalFlash_BootDataSec_Size;
            Info.boot_sec.para_size = 0;
            Info.boot_sec.para_num = 0;
            tab_addr_offset = (Info.boot_sec.tab_addr + Info.boot_sec.tab_size) + Storage_ReserveBlock_Size;

            Info.sys_sec.tab_addr = tab_addr_offset;
            Info.sys_sec.tab_size = page_num * OnChipFlash_Storage_TabSize;
            Info.sys_sec.data_sec_size = InternalFlash_SysDataSec_Size;
            Info.sys_sec.page_num = page_num;
            Info.sys_sec.para_size = 0;
            Info.sys_sec.para_num = 0;
            tab_addr_offset += Info.sys_sec.tab_size + Storage_ReserveBlock_Size;
                
            Info.user_sec.tab_addr = tab_addr_offset;
            Info.user_sec.tab_size = page_num * OnChipFlash_Storage_TabSize;
            Info.user_sec.data_sec_size = InternalFlash_UserDataSec_Size;
            Info.user_sec.page_num = page_num;
            Info.user_sec.para_size = 0;
            Info.user_sec.para_num = 0;
            tab_addr_offset += Info.user_sec.tab_size + Storage_ReserveBlock_Size;

            /* get the remaining size of rom space has left */
            if(Info.total_size < (tab_addr_offset - BaseInfo_start_addr))
                return false;
            
            remain_data_sec_size = Info.total_size - (tab_addr_offset - BaseInfo_start_addr);
            data_sec_size += Info.boot_sec.data_sec_size + Storage_ReserveBlock_Size;
            data_sec_size += Info.sys_sec.data_sec_size + Storage_ReserveBlock_Size;
            data_sec_size += Info.user_sec.data_sec_size + Storage_ReserveBlock_Size;

            if(remain_data_sec_size < data_sec_size)
                return false;

            Info.remain_size = remain_data_sec_size - data_sec_size;
            Info.data_sec_size = Info.boot_sec.data_sec_size + Info.sys_sec.data_sec_size + Info.user_sec.data_sec_size;

            /* get data sec addr */
            Info.boot_sec.data_sec_addr = tab_addr_offset;
            tab_addr_offset += InternalFlash_BootDataSec_Size;
            tab_addr_offset += Storage_ReserveBlock_Size;

            Info.sys_sec.data_sec_addr = tab_addr_offset;
            tab_addr_offset += InternalFlash_SysDataSec_Size;
            tab_addr_offset += Storage_ReserveBlock_Size;

            Info.user_sec.data_sec_addr = tab_addr_offset;
            tab_addr_offset += InternalFlash_UserDataSec_Size;
            tab_addr_offset += Storage_ReserveBlock_Size;

            Storage_Monitor.internal_info = Info;
            addr_offset = From_Start_Address;
            break;

        /* still in developping */
        case External_Flash:
            info_page_size = ExtFlash_Storage_InfoPageSize;
            StorageIO_API = &ExternalFlash_IO;
            memcpy(Info.tag, EXTERNAL_STORAGE_PAGE_TAG, EXTERNAL_PAGE_TAG_SIZE);

            if (Storage_Monitor.ExtDev_ptr)
                sector_size = ((Storage_ExtFLashDevObj_TypeDef *)(Storage_Monitor.ExtDev_ptr))->sector_size;
            
            if ((StorageIO_API->erase == NULL) || \
                (StorageIO_API->read  == NULL) || \
                (StorageIO_API->write == NULL))
                return false;
            
            Info.total_size = ExtFlash_Storage_TotalSize;
            Info.base_addr = Storage_Monitor.external_info.base_addr;
            
            BaseInfo_start_addr = Info.base_addr;
            page_num = Storage_ExtFlash_Max_Capacity / (ExtFlash_Storage_TabSize / StorageItem_Size);
            if(page_num == 0)
                return false;
            
            Info.boot_sec.tab_addr = BaseInfo_start_addr + info_page_size;
            Info.boot_sec.tab_size = BootSection_Block_Size * BootTab_Num;
            Info.boot_sec.page_num = BootTab_Num;
            Info.boot_sec.data_sec_size = ExternalFlash_BootDataSec_Size;
            Info.boot_sec.para_size = 0;
            Info.boot_sec.para_num = 0;
            tab_addr_offset = (Info.boot_sec.tab_addr + Info.boot_sec.tab_size) + Storage_ReserveBlock_Size;

            Info.sys_sec.tab_addr = tab_addr_offset;
            Info.sys_sec.tab_size = page_num * ExtFlash_Storage_TabSize;
            Info.sys_sec.data_sec_size = ExternalFlash_SysDataSec_Size;
            Info.sys_sec.page_num = page_num;
            Info.sys_sec.para_size = 0;
            Info.sys_sec.para_num = 0;
            tab_addr_offset += Info.sys_sec.tab_size + Storage_ReserveBlock_Size;
                
            Info.user_sec.tab_addr = tab_addr_offset;
            Info.user_sec.tab_size = page_num * ExtFlash_Storage_TabSize;
            Info.user_sec.data_sec_size = ExternalFlash_UserDataSec_Size;
            Info.user_sec.page_num = page_num;
            Info.user_sec.para_size = 0;
            Info.user_sec.para_num = 0;
            tab_addr_offset += Info.user_sec.tab_size + Storage_ReserveBlock_Size;
            
            /* get the remaining size of rom space has left */
            if(Info.total_size < (tab_addr_offset - BaseInfo_start_addr))
                return false;
            
            remain_data_sec_size = Info.total_size - (tab_addr_offset - BaseInfo_start_addr);
            data_sec_size += Info.boot_sec.data_sec_size + Storage_ReserveBlock_Size;
            data_sec_size += Info.sys_sec.data_sec_size + Storage_ReserveBlock_Size;
            data_sec_size += Info.user_sec.data_sec_size + Storage_ReserveBlock_Size;

            if(remain_data_sec_size < data_sec_size)
                return false;
                
            Info.remain_size = remain_data_sec_size - data_sec_size;
            Info.data_sec_size = Info.boot_sec.data_sec_size + Info.sys_sec.data_sec_size + Info.user_sec.data_sec_size;

            /* get data sec addr */
            Info.boot_sec.data_sec_addr = tab_addr_offset;
            tab_addr_offset += ExternalFlash_BootDataSec_Size;
            tab_addr_offset += Storage_ReserveBlock_Size;

            Info.sys_sec.data_sec_addr = tab_addr_offset;
            tab_addr_offset += ExternalFlash_SysDataSec_Size;
            tab_addr_offset += Storage_ReserveBlock_Size;

            Info.user_sec.data_sec_addr = tab_addr_offset;
            tab_addr_offset += ExternalFlash_UserDataSec_Size;
            tab_addr_offset += Storage_ReserveBlock_Size;

            Storage_Monitor.external_info = Info;
            addr_offset = BaseInfo_start_addr - Storage_Monitor.external_info.base_addr;
            break;

        default:
            return false;
    }

    /* write 0 to info section */
    memset(page_data_tmp, 0, info_page_size);

    /* read out and erase sector */
    if (!StorageIO_API->read(addr_offset, page_data_tmp, sizeof(Info)))
        return false;

    /* write base info to info section */
    memcpy(page_data_tmp, &Info, sizeof(Info));
    crc = Common_CRC16(page_data_tmp, info_page_size - sizeof(crc));
    memcpy(&page_data_tmp[info_page_size - sizeof(crc)], &crc, sizeof(crc));

    /* write into flash chip */
    if (!StorageIO_API->write(addr_offset, page_data_tmp, info_page_size))
        return false;

    /* read out again */
    if (!StorageIO_API->read(addr_offset, page_data_tmp, sizeof(Info)))
        return false;

    /* compare with target */
    memcpy(&Info_Rx, page_data_tmp, sizeof(Info_Rx));
    if (memcmp(&Info_Rx, &Info, sizeof(Storage_FlashInfo_TypeDef)) != 0)
        return false;

    if (!Storage_Establish_Tab(type, Para_Boot) || \
        !Storage_Establish_Tab(type, Para_Sys)  || \
        !Storage_Establish_Tab(type, Para_User))
        return false;

    return true;
}

static storage_handle Storage_Search(Storage_MediumType_List medium, Storage_ParaClassType_List class, const char *name)
{
    uint32_t base_addr = 0;
    storage_handle hdl = 0;
    StorageIO_TypeDef *StorageIO_API = NULL;
    Storage_FlashInfo_TypeDef *info = NULL;
    Storage_BaseSecInfo_TypeDef *sec = NULL;

    if((name == NULL) || (strlen(name) == 0))
        return 0;

    if(medium == Internal_Flash)
    {
        StorageIO_API = &InternalFlash_IO;
        info = &Storage_Monitor.internal_info;
    }
    else
    {
        StorageIO_API = &ExternalFlash_IO;
        info = &Storage_Monitor.external_info;
    }

    sec = Storage_Get_SecInfo(info, class);
    if (sec == NULL)
        return 0;
    
    base_addr = sec->tab_addr;

    return hdl;
}

static Storage_BaseSecInfo_TypeDef* Storage_Get_SecInfo(Storage_FlashInfo_TypeDef *info, Storage_ParaClassType_List class)
{
    if (info)
    {
        switch(class)
        {
            case Para_Boot: return &(info->boot_sec);
            case Para_Sys:  return &(info->sys_sec);
            case Para_User: return &(info->user_sec);
            default:        return NULL;
        }
    }

    return NULL;
}

/************************************************** External Flash IO API Section ************************************************/
static bool Storage_External_Chip_W25Qxx_SelectPin_Ctl(bool state)
{
    BspGPIO.write(ExtFlash_CS_Pin, state);
    return true;
}

static uint16_t Storage_External_Chip_W25Qxx_BusTx(uint8_t *p_data, uint16_t len, uint32_t time_out)
{
    BspSPI_NorModeConfig_TypeDef *p_cfg = To_NormalSPI_ObjPtr(Storage_Monitor.ExtBusCfg_Ptr);

    if (p_data && len && p_cfg && p_cfg->Instance)
    {
        if (ExtFlash_Bus_Api.trans(p_cfg->Instance, p_data, len, time_out))
            return len;
    }

    return 0;
}

static uint16_t Storage_External_Chip_W25Qxx_BusRx(uint8_t *p_data, uint16_t len, uint32_t time_out)
{
    BspSPI_NorModeConfig_TypeDef *p_cfg = To_NormalSPI_ObjPtr(Storage_Monitor.ExtBusCfg_Ptr);

    if (p_data && len && p_cfg && p_cfg->Instance)
    {
        if (ExtFlash_Bus_Api.receive(p_cfg->Instance, p_data, len, time_out))
            return len;
    }

    return 0;
}

static uint16_t Storage_External_Chip_W25Qxx_BusTrans(uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t time_out)
{
    BspSPI_NorModeConfig_TypeDef *p_cfg = To_NormalSPI_ObjPtr(Storage_Monitor.ExtBusCfg_Ptr);

    if (tx && rx && len && p_cfg && p_cfg->Instance)
    {
        if (ExtFlash_Bus_Api.trans_receive(p_cfg->Instance, tx, rx, len, time_out))
            return len;
    }

    return 0;
}

static bool Storage_ExtFlash_Read(uint32_t addr_offset, uint8_t *p_data, uint32_t len)
{
    uint32_t read_addr = 0;
    uint32_t flash_end_addr = 0;
    Storage_ExtFLashDevObj_TypeDef *dev = NULL;

    if ((Storage_Monitor.ExtDev_ptr != NULL) && p_data && len)
    {
        read_addr = Storage_Monitor.external_info.base_addr + addr_offset;       
        dev = (Storage_ExtFLashDevObj_TypeDef *)(Storage_Monitor.ExtDev_ptr);

        switch((uint8_t)dev->chip_type)
        {
            case Storage_ChipType_W25Qxx:
                if (dev->dev_api && dev->dev_obj)
                {
                    /* get w25qxx device info */
                    /* address check */
                    flash_end_addr = To_DevW25Qxx_API(dev->dev_api)->info(To_DevW25Qxx_OBJ(dev->dev_obj)).start_addr;
                    if (flash_end_addr > read_addr)
                        return false;

                    /* range check */
                    flash_end_addr += To_DevW25Qxx_API(dev->dev_api)->info(To_DevW25Qxx_OBJ(dev->dev_obj)).flash_size;
                    if ((len + read_addr) > flash_end_addr)
                        return false;

                    /* W25Qxx device read */
                    if (To_DevW25Qxx_API(dev->dev_api)->read(To_DevW25Qxx_OBJ(dev->dev_obj), read_addr, p_data, len) == DevW25Qxx_Ok)
                        return true;
                }
                break;

            default:
                return false;
        }
    }

    return false;
}

static bool Storage_ExtFlash_Write(uint32_t addr_offset, uint8_t *p_data, uint32_t len)
{
    uint32_t write_addr = 0;
    uint32_t flash_end_addr = 0;
    uint32_t section_start_addr = 0;
    uint32_t section_size = 0;
    uint32_t write_offset = 0;
    Storage_ExtFLashDevObj_TypeDef *dev = NULL;
    DevW25Qxx_Error_List state;

    if ((Storage_Monitor.ExtDev_ptr != NULL) && p_data && len)
    {
        write_addr = Storage_Monitor.external_info.base_addr + addr_offset;
        dev = (Storage_ExtFLashDevObj_TypeDef *)(Storage_Monitor.ExtDev_ptr);
    
        switch((uint8_t)dev->chip_type)
        {
            case Storage_ChipType_W25Qxx:
                if (dev->dev_api && dev->dev_obj)
                {
                    section_size = To_DevW25Qxx_API(dev->dev_api)->info(To_DevW25Qxx_OBJ(dev->dev_obj)).sector_size;
                    /* get w25qxx device info */
                    /* address check */
                    flash_end_addr = To_DevW25Qxx_API(dev->dev_api)->info(To_DevW25Qxx_OBJ(dev->dev_obj)).start_addr;
                    if (flash_end_addr > write_addr)
                        return false;

                    /* range check */
                    flash_end_addr += To_DevW25Qxx_API(dev->dev_api)->info(To_DevW25Qxx_OBJ(dev->dev_obj)).flash_size;
                    if ((len + write_addr) > flash_end_addr)
                        return false;
                    
                    if (section_size)
                    {
                        if (section_size && (section_size >= len))
                        {
                            section_start_addr = To_DevW25Qxx_API(dev->dev_api)->get_section_start_addr(To_DevW25Qxx_OBJ(dev->dev_obj), write_addr);
                            write_offset = write_addr - section_start_addr;
                            if (section_size > sizeof(page_data_tmp))
                                return false;

                            if ((write_addr + len) <= (section_start_addr + section_size))
                            {
                                /* circumstances 1: store data size less than flash sector size and only none multiple sector write is needed */
                                /* read whole section */
                                if (To_DevW25Qxx_API(dev->dev_api)->read(To_DevW25Qxx_OBJ(dev->dev_obj), section_start_addr, page_data_tmp, section_size) != DevW25Qxx_Ok)
                                    return false;

                                /* erase whole section */
                                if (!To_DevW25Qxx_API(dev->dev_api)->erase_sector(To_DevW25Qxx_OBJ(dev->dev_obj), section_start_addr) != DevW25Qxx_Ok)
                                    return false;

                                /* copy data to section data read out */
                                memcpy(page_data_tmp + write_offset, p_data, len);

                                /* update whole section */
                                state = To_DevW25Qxx_API(dev->dev_api)->write(To_DevW25Qxx_OBJ(dev->dev_obj), section_start_addr, page_data_tmp, section_size);
                                memset(page_data_tmp, 0, section_size);

                                if (state == DevW25Qxx_Ok)
                                    return true;
                            }
                            else
                            {
                                /* circumstances 2: store data size less than flash sector length but need to write from the end of the sector N to the start of the sector N + 1 */
                                /* need to operate two sections */
                                for (uint8_t i = 0; i < 2; i++)
                                {
                                    /* read whole section */
                                    if (To_DevW25Qxx_API(dev->dev_api)->read(To_DevW25Qxx_OBJ(dev->dev_obj), section_start_addr, page_data_tmp, section_size) != DevW25Qxx_Ok)
                                        return false;

                                    /* erase whole section */
                                    if (!To_DevW25Qxx_API(dev->dev_api)->erase_sector(To_DevW25Qxx_OBJ(dev->dev_obj), section_start_addr) != DevW25Qxx_Ok)
                                        return false;

                                    /* copy data to section data read out */
                                    memcpy(page_data_tmp + write_offset, p_data, len);
                                    /* update whole section */
                                    state = To_DevW25Qxx_API(dev->dev_api)->write(To_DevW25Qxx_OBJ(dev->dev_obj), section_start_addr, page_data_tmp, section_size);
                                    /* clear cache buff */
                                    memset(page_data_tmp, 0, section_size);
                                    /* update target section address */
                                    section_start_addr += section_size;
                                    /* reupdate target section address */
                                    section_start_addr = To_DevW25Qxx_API(dev->dev_api)->get_section_start_addr(To_DevW25Qxx_OBJ(dev->dev_obj), section_start_addr);
                                    p_data += section_size - write_addr;
                                    len -= section_size - write_addr;
                                    write_offset = 0;
                                }
                                    
                                if (state == DevW25Qxx_Ok)
                                    return true;
                            }
                        }
                        else if (section_size < len)
                        {
                            /* circumstances 3: store data size large than flash sector length */
                        }
                    }
                    else
                        return false;
                }
                break;

            default:
                return false;
        }
    }

    return false;
}

static bool Storage_ExtFlash_Erase(uint32_t addr_offset, uint32_t len)
{
    uint32_t erase_start_addr = 0;
    Storage_ExtFLashDevObj_TypeDef *dev = NULL;

    if ((Storage_Monitor.ExtDev_ptr != NULL) && len)
    {
        /* erase external flash sector */
        erase_start_addr = Storage_Monitor.external_info.base_addr + addr_offset;
        dev = (Storage_ExtFLashDevObj_TypeDef *)(Storage_Monitor.ExtDev_ptr);
    
        switch((uint8_t)dev->chip_type)
        {
            case Storage_ChipType_W25Qxx:
                if (dev->dev_api && dev->dev_obj)
                {
                    /* get w25qxx device info */
                    /* address check */
                    if (erase_start_addr < To_DevW25Qxx_API(dev->dev_api)->info(To_DevW25Qxx_OBJ(dev->dev_obj)).start_addr)
                        return false;

                    /* W25Qxx device read */
                    if (To_DevW25Qxx_API(dev->dev_api)->erase_sector(To_DevW25Qxx_OBJ(dev->dev_obj), erase_start_addr) == DevW25Qxx_Ok)
                        return true;
                }
                break;

            default:
                return false;
        }
    }
    
    return false;
}

static bool Storage_ExtFlash_EraseAll(void)
{
    return false;
}

/************************************************** Internal Flash IO API Section ************************************************/
static bool Storage_OnChipFlash_Read(uint32_t addr_offset, uint8_t *p_data, uint32_t len)
{
    uint32_t addr = Storage_Monitor.internal_info.base_addr + addr_offset;
    uint32_t read_size = 0;
    uint8_t read_cnt = 1;

    if(len > OnChipFlash_MaxRWSize)
    {
        read_cnt = len / OnChipFlash_MaxRWSize;
        read_size = OnChipFlash_MaxRWSize;
        
        if(len % OnChipFlash_MaxRWSize)
            read_cnt ++;
    }
    else
        read_size = len;

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
    uint32_t write_addr = Storage_Monitor.internal_info.base_addr + addr_offset;
    
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
    uint32_t addr = Storage_Monitor.internal_info.base_addr + addr_offset;
    
    /* erase only */
    return BspFlash.erase(addr, len);
}

/************************************************** Shell API Section ************************************************/
static void Storage_SecInfo_Print(Shell *obj, Storage_BaseSecInfo_TypeDef *p_SecInfo)
{
    if(obj == NULL)
        return;

    shellPrint(obj, "\t\taddr:            %x\r\n", p_SecInfo->tab_addr);
    shellPrint(obj, "\t\tpage num:        %d\r\n", p_SecInfo->page_num);
    shellPrint(obj, "\t\tsize:            %d\r\n", p_SecInfo->tab_size);
    shellPrint(obj, "\t\tdata sec addr:   %x\r\n", p_SecInfo->data_sec_addr);
    shellPrint(obj, "\t\tdata sec size:   %d\r\n", p_SecInfo->data_sec_size);
    shellPrint(obj, "\t\tfree slot addr:  %x\r\n", p_SecInfo->free_slot_addr);
    shellPrint(obj, "\t\tparam num:       %d\r\n", p_SecInfo->para_num);
    shellPrint(obj, "\t\tparam size:      %d\r\n", p_SecInfo->para_size);
}

static const char* Storage_Error_Print(Storage_ErrorCode_List code)
{
    switch((uint8_t) code)
    {
        case Storage_Error_None:
            return Storage_ErrorCode_ToStr(Storage_Error_None);
        
        case Storage_ModuleInit_Error:
            return Storage_ErrorCode_ToStr(Storage_ModuleInit_Error);

        case Storage_Read_Error:
            return Storage_ErrorCode_ToStr(Storage_Read_Error);

        case Storage_Write_Error:
            return Storage_ErrorCode_ToStr(Storage_Write_Error);

        case Storage_Erase_Error:
            return Storage_ErrorCode_ToStr(Storage_Erase_Error);

        case Storage_NameMatched:
            return Storage_ErrorCode_ToStr(Storage_NameMatched);

        case Storage_SlotHeader_Error:
            return Storage_ErrorCode_ToStr(Storage_SlotHeader_Error);

        case Storage_ExternalFlash_NotAvailable:
            return Storage_ErrorCode_ToStr(Storage_ExternalFlash_NotAvailable);

        case Storage_InternalFlash_NotAvailable:
            return Storage_ErrorCode_ToStr(Storage_InternalFlash_NotAvailable);

        case Storage_Class_Error:
            return Storage_ErrorCode_ToStr(Storage_Class_Error);

        case Storage_RW_Api_Error:
            return Storage_ErrorCode_ToStr(Storage_RW_Api_Error);

        case Storage_DataInfo_Error:
            return Storage_ErrorCode_ToStr(Storage_DataInfo_Error);

        case Storage_DataSize_Overrange:
            return Storage_ErrorCode_ToStr(Storage_DataSize_Overrange);

        case Storage_BaseInfo_Updata_Error:
            return Storage_ErrorCode_ToStr(Storage_BaseInfo_Updata_Error);

        case Storage_FreeSlot_Update_Error:
            return Storage_ErrorCode_ToStr(Storage_FreeSlot_Update_Error);

        default:
            return "Unknow Error\r\n";
    }
}

static void Storage_MediumType_Print(Shell *obj)
{
    if(obj == NULL)
        return;
    
    shellPrint(obj, "\tmedium para\r\n");
    shellPrint(obj, "\tInternal_Flash ---- %d\r\n", Internal_Flash);
    shellPrint(obj, "\tExternal_Flash ---- %d\r\n", External_Flash);
    shellPrint(obj, "\r\n");
}

static void Storage_ClassType_Print(Shell *obj)
{
    if(obj == NULL)
        return;
    
    shellPrint(obj, "\tclass para\r\n");
    shellPrint(obj, "\tPara_Boot ---- %d\r\n", Para_Boot);
    shellPrint(obj, "\tPara_Sys  ---- %d\r\n", Para_Sys);
    shellPrint(obj, "\tPara_User ---- %d\r\n", Para_User);
    shellPrint(obj, "\r\n");
}

static bool Storage_SelectedClass_Print(Shell *obj, Storage_ParaClassType_List class)
{
    if(obj == NULL)
        return false;

    switch((uint8_t) class)
    {
        case Para_Boot:
            shellPrint(obj, "\t[Boot Section Selected]\r\n");
            break;
        
        case Para_Sys:
            shellPrint(obj, "\t[Sys Section Selected]\r\n");
            break;
        
        case Para_User:
            shellPrint(obj, "\t[User Section Selected]\r\n");
            break;

        default:
            shellPrint(obj, "\tClass arg error\r\n");
            return false;
    }
}

static void Storage_Shell_Get_BaseInfo(Storage_MediumType_List medium)
{
    Shell *shell_obj = Shell_GetInstence();
    Storage_FlashInfo_TypeDef *p_Flash = NULL;

    if(shell_obj == NULL)
        return;
    
    shellPrint(shell_obj, "\t[Storage Shell] Get BaseInfo\r\n");
    if(!Storage_Monitor.init_state)
    {
        shellPrint(shell_obj, "\thalt by init state\r\n");
        return;
    }

    Storage_MediumType_Print(shell_obj);
    
    switch((uint8_t) medium)
    {
        case Internal_Flash:
            shellPrint(shell_obj, "\t\t[Internal_Flash Selected]\r\n");
            p_Flash = &Storage_Monitor.internal_info;
            if(memcmp(p_Flash->tag, INTERNAL_STORAGE_PAGE_TAG, INTERNAL_PAGE_TAG_SIZE))
            {
                shellPrint(shell_obj, "\t\tInternal_Flash Info Error\r\n");
                return;
            }
            shellPrint(shell_obj, "\t\ttotal   size:\t%d\r\n", p_Flash->total_size);
            shellPrint(shell_obj, "\t\tremain  size:\t%d\r\n", p_Flash->remain_size);
            shellPrint(shell_obj, "\t\tstorage size:\t%d\r\n", p_Flash->data_sec_size);
            break;
        
        case External_Flash:
            shellPrint(shell_obj, "\t\tExternal_Flash Selected\r\n");
            shellPrint(shell_obj, "\t\tStill in developping\r\n");
            shellPrint(shell_obj, "\t\treturn\r\n");
            p_Flash = &Storage_Monitor.external_info;
            return;

        default:
            shellPrint(shell_obj, "medium para error\r\n");
            return;
    }

    /* print boot info */
    shellPrint(shell_obj, "\r\n\t[Boot Section]\r\n");
    Storage_SecInfo_Print(shell_obj, &p_Flash->boot_sec);

    /* print sys  info */
    shellPrint(shell_obj, "\r\n\t[System Section]\r\n");
    Storage_SecInfo_Print(shell_obj, &p_Flash->sys_sec);

    /* print user info */
    shellPrint(shell_obj, "\r\n\t[User Section]\r\n");
    Storage_SecInfo_Print(shell_obj, &p_Flash->user_sec);
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, Storage_BaseInfo, Storage_Shell_Get_BaseInfo, Storage base info);

static void Storage_Dump(Storage_MediumType_List medium, Storage_ParaClassType_List class)
{
    Shell *shell_obj = Shell_GetInstence();
    Storage_ErrorCode_List error_code = Storage_Error_None;
    
    if(shell_obj == NULL)
        return;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, Storage_Dump, Storage_Dump, Dump Selected Class Data In Storage Section);

static void Storage_Test(Storage_MediumType_List medium, Storage_ParaClassType_List class, char *test_name, char *test_data)
{
    Shell *shell_obj = Shell_GetInstence();
    Storage_ErrorCode_List error_code = Storage_Error_None;
    
    if(shell_obj == NULL)
        return;

    shellPrint(shell_obj, "\t[Storage Shell] Data Storage Test\r\n");
    if(!Storage_Monitor.init_state)
    {
        shellPrint(shell_obj, "\thalt by init state\r\n");
        return;
    }

    Storage_MediumType_Print(shell_obj);
    
    if(medium >= External_Flash)
    {
        if(medium == External_Flash)
        {
            shellPrint(shell_obj, "\tExternal flash still in developping\r\n");
        }
        else
            shellPrint(shell_obj, "\tstorage medium arg error\r\n");
        
        return;
    }
    else
    {
        shellPrint(shell_obj, "\t[Internal_Flash Selected]\r\n");
    }

    Storage_ClassType_Print(shell_obj);
    if(class > Para_User)
    {
        shellPrint(shell_obj, "\tstorage class arg error\r\n");
        return;
    }

    Storage_SelectedClass_Print(shell_obj, class);
    if( (test_name == NULL) || \
        (test_data == NULL) || \
        (strlen(test_name) == 0) || \
        (strlen(test_data) == 0))
    {
        shellPrint(shell_obj, "\t storage name or storage data error \r\n");
        shellPrint(shell_obj, "\t exp :\r\n");
        shellPrint(shell_obj, "\t\t Storage 0 0 test_name test_data\r\n");
    }

    shellPrint(shell_obj, "\tStorage Name: %s\r\n", test_name);
    shellPrint(shell_obj, "\tStorage Data: %s\r\n", test_data);
    shellPrint(shell_obj, "\tStorage Size: %d\r\n", strlen(test_data));

    error_code = Storage_CreateItem(medium, class, test_name, test_data, strlen(test_data));
    if(error_code != Storage_Error_None)
    {
        shellPrint(shell_obj, "\t[Storage Test Failed]\r\n");
        shellPrint(shell_obj, "\t[Error_Code]: %s\r\n", Storage_Error_Print(error_code));
    }
    else
        shellPrint(shell_obj, "\t[Storage Test Done]\r\n");
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, Storage_Test, Storage_Test, Data Storage Test);

static void Storage_Show_Tab(Storage_MediumType_List medium, Storage_ParaClassType_List class)
{
    Shell *shell_obj = Shell_GetInstence();
    
    if(shell_obj == NULL)
        return;
    
    shellPrint(shell_obj, "[Storage Shell] Show Storage Tab\r\n");
    if(!Storage_Monitor.init_state)
    {
        shellPrint(shell_obj, "\thalt by init state\r\n");
        return;
    }
    
    switch((uint8_t) medium)
    {
        case Internal_Flash:
            break;

        case External_Flash:
        default:
            return;
    }

    switch((uint8_t) class)
    {
        case Para_Boot:
            break;

        case Para_Sys:
            break;
        
        case Para_User:
            break;
        
        default:
            return;
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, Storage_ShowTab, Storage_Show_Tab, Storage show Tab);
