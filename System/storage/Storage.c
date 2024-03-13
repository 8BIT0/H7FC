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
#include "error_log.h"

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
static uint8_t page_data_tmp[Storage_TabSize * 2] __attribute__((aligned(4))) = {0};
static uint8_t flash_write_tmp[Storage_TabSize * 2] __attribute__((aligned(4))) = {0};
static uint8_t flash_read_tmp[Storage_TabSize * 2] __attribute__((aligned(4))) = {0};

static bool Storage_OnChipFlash_Read(uint32_t addr_offset, uint8_t *p_data, uint32_t len);
static bool Storage_OnChipFlash_Write(uint32_t addr_offset, uint8_t *p_data, uint32_t len);
static bool Storage_OnChipFlash_Erase(uint32_t addr_offset, uint32_t len);

static bool Storage_Clear_Tab(StorageIO_TypeDef *storage_api, uint32_t addr, uint32_t tab_num);
static bool Storage_Establish_Tab(Storage_MediumType_List type, Storage_ParaClassType_List class);

static bool Storage_ExtFlash_Read(uint32_t addr_offset, uint8_t *p_data, uint32_t len);
static bool Storage_ExtFlash_Write(uint32_t addr_offset, uint8_t *p_data, uint32_t len);
static bool Storage_ExtFlash_Erase(uint32_t addr_offset, uint32_t len);
static bool Storage_ExtFlash_EraseAll(void);

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
static bool Storage_Compare_ItemSlot_CRC(const Storage_Item_TypeDef item);
static bool Storage_Comput_ItemSlot_CRC(Storage_Item_TypeDef *p_item);
static Storage_BaseSecInfo_TypeDef* Storage_Get_SecInfo(Storage_FlashInfo_TypeDef *info, Storage_ParaClassType_List class);
static bool Storage_DeleteSingalDataSlot(uint32_t slot_addr, uint8_t *p_data, Storage_BaseSecInfo_TypeDef *p_Sec, StorageIO_TypeDef *StorageIO_API);
static Storage_ErrorCode_List Storage_FreeSlot_CheckMerge(uint32_t slot_addr, Storage_FreeSlot_TypeDef *slot_info, Storage_BaseSecInfo_TypeDef *p_Sec, StorageIO_TypeDef *StorageIO_API);

/* external function */
static bool Storage_Init(Storage_ModuleState_TypeDef enable, Storage_ExtFLashDevObj_TypeDef *ExtDev);
static Storage_Item_TypeDef Storage_Search(Storage_MediumType_List type, Storage_ParaClassType_List class, const char *name);
static Storage_ErrorCode_List Storage_DeleteItem(Storage_MediumType_List type, Storage_ParaClassType_List class, const char *name, uint32_t size);
static Storage_ErrorCode_List Storage_CreateItem(Storage_MediumType_List type, Storage_ParaClassType_List class, const char *name, uint8_t *p_data, uint32_t size);
static Storage_ErrorCode_List Storage_SlotData_Update(Storage_MediumType_List type, Storage_ParaClassType_List class, storage_handle data_slot_hdl, uint8_t *p_data, uint16_t size);

Storage_TypeDef Storage = {
    .init = Storage_Init,
};

static bool Storage_Init(Storage_ModuleState_TypeDef enable, Storage_ExtFLashDevObj_TypeDef *ExtDev)
{
    void *ext_flash_bus_cfg = NULL;
    uint8_t extmodule_init_state;
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
                        Storage_Monitor.InternalFlash_BuildTab_cnt ++;
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
                        ExtDev->dev_obj = SrvOsCommon.malloc(sizeof(DevW25QxxObj_TypeDef));
                        if (ExtDev->dev_obj)
                        {
                            /* set get time callback */
                            To_DevW25Qxx_OBJ(ExtDev->dev_obj)->systick = SrvOsCommon.get_os_ms;

                            /* set bus control callback */
                            To_DevW25Qxx_OBJ(ExtDev->dev_obj)->cs_ctl = Storage_External_Chip_W25Qxx_SelectPin_Ctl;
                            To_DevW25Qxx_OBJ(ExtDev->dev_obj)->bus_tx = Storage_External_Chip_W25Qxx_BusTx;
                            To_DevW25Qxx_OBJ(ExtDev->dev_obj)->bus_rx = Storage_External_Chip_W25Qxx_BusRx;
                            To_DevW25Qxx_OBJ(ExtDev->dev_obj)->bus_trans = Storage_External_Chip_W25Qxx_BusTrans;
                            Storage_Monitor.ExtDev_ptr = ExtDev;
                            Storage_Monitor.ExternalFlash_ReInit_cnt = ExternalModule_ReInit_Cnt;

reinit_external_flash_module:
                            extmodule_init_state = To_DevW25Qxx_API(ExtDev->dev_api)->init(To_DevW25Qxx_OBJ(ExtDev->dev_obj));
                            Storage_Monitor.module_prod_type = To_DevW25Qxx_API(ExtDev->dev_api)->info(To_DevW25Qxx_OBJ(ExtDev->dev_obj)).prod_type;
                            Storage_Monitor.module_prod_code = To_DevW25Qxx_API(ExtDev->dev_api)->info(To_DevW25Qxx_OBJ(ExtDev->dev_obj)).prod_code;

                            if (extmodule_init_state == DevW25Qxx_Ok)
                            {
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
                                                Storage_Monitor.ExternalFlash_BuildTab_cnt ++;
                                                Storage_Monitor.module_init_reg.bit.external = true;

                                                /* after tab builded read storage info again */
                                                goto reupdate_external_flash_info;
                                            }
                                        }
                                    }
                                }
                                else
                                {
                                    Storage_Monitor.module_init_reg.bit.external = true;
                                }
                            }
                            else
                            {
                                if (Storage_Monitor.ExternalFlash_ReInit_cnt)
                                {
                                    Storage_Monitor.ExternalFlash_ReInit_cnt --;
                                    goto reinit_external_flash_module;
                                }

                                Storage_Monitor.ExternalFlash_Error_Code = Storage_ModuleInit_Error;
                            }
                        }
                        else
                            Storage_Monitor.ExternalFlash_Error_Code = Storage_ExtDevObj_Error;
                    }
                    else
                        Storage_Monitor.ExternalFlash_Error_Code = Storage_ModuleType_Error;
                }
                else
                    Storage_Monitor.ExternalFlash_Error_Code = Storage_BusInit_Error;
            }
            else
                Storage_Monitor.ExternalFlash_Error_Code = Storage_BusCfg_Malloc_Failed;
        }
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

            if (!StorageIO_API->erase(addr_offset, size))
                return false;

            if (!StorageIO_API->read(addr_offset, page_data_tmp, size))
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
                return false;

            FreeSlot_Info = (Storage_FreeSlot_TypeDef *)page_data_tmp;

            if ((FreeSlot_Info->head_tag != STORAGE_SLOT_HEAD_TAG) || \
                (FreeSlot_Info->end_tag != STORAGE_SLOT_END_TAG))
                return false;

            free_i ++;
            if (FreeSlot_Info->nxt_addr == 0)
                break;

            free_slot_addr = FreeSlot_Info->nxt_addr;
        }
        
        if (sec_info->para_num)
        {
            tab_addr = sec_info->tab_addr;

            for (uint16_t tab_i = 0; tab_i < sec_info->page_num; tab_i ++)
            {
                if (!storage_api->read(tab_addr, page_data_tmp, sec_info->tab_size / sec_info->page_num))
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
                        crc_buf = (uint8_t *)&p_ItemList[item_i] + sizeof(p_ItemList[item_i].head_tag);
                        crc_len = sizeof(Storage_Item_TypeDef);
                        crc_len -= sizeof(p_ItemList[item_i].head_tag);
                        crc_len -= sizeof(p_ItemList[item_i].end_tag);
                        crc_len -= sizeof(p_ItemList[item_i].crc16);
                        store_param_size += p_ItemList[item_i].len;
                        
                        crc16 = Common_CRC16(crc_buf, crc_len);
                        if (crc16 != p_ItemList[item_i].crc16)
                            return false;

                        store_param_found ++;
                    }
                }

                tab_addr += (sec_info->tab_size / sec_info->page_num);
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
    Storage_FlashInfo_TypeDef Info_r;
    uint16_t i = 0;
    uint16_t crc = 0;
    uint16_t crc_read = 0;
    char flash_tag[INTERNAL_PAGE_TAG_SIZE + EXTERNAL_PAGE_TAG_SIZE];

    memset(flash_tag, '\0', sizeof(flash_tag));
    memset(&Info_r, 0, sizeof(Storage_FlashInfo_TypeDef));

    switch((uint8_t)type)
    {
        case Internal_Flash:
            StorageIO_API = &InternalFlash_IO;
            memcpy(flash_tag, INTERNAL_STORAGE_PAGE_TAG, INTERNAL_PAGE_TAG_SIZE);
            p_Info = &Storage_Monitor.internal_info;
            break;

        /* still in developping */
        case External_Flash:
            StorageIO_API = &ExternalFlash_IO;
            memcpy(flash_tag, EXTERNAL_STORAGE_PAGE_TAG, EXTERNAL_PAGE_TAG_SIZE);
            p_Info = &Storage_Monitor.external_info;
            break;

        default:
            return false;
    }
    
    if (StorageIO_API->read(From_Start_Address, page_data_tmp, Storage_TabSize))
    {
        /* check internal storage tag */
        Info_r = *(Storage_FlashInfo_TypeDef *)page_data_tmp;
        
        /* check storage tag */
        /* check boot / sys / user  start addr */
        if ((strcmp(Info_r.tag, flash_tag) != 0) || \
            (Info_r.boot_sec.tab_addr == 0) || \
            (Info_r.sys_sec.tab_addr == 0) || \
            (Info_r.user_sec.tab_addr == 0) || \
            (Info_r.boot_sec.tab_addr == Info_r.sys_sec.tab_addr) || \
            (Info_r.boot_sec.tab_addr == Info_r.user_sec.tab_addr) || \
            (Info_r.sys_sec.tab_addr == Info_r.user_sec.tab_addr))
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
        if (Storage_Check_Tab(StorageIO_API, &Info_r.boot_sec) && \
            Storage_Check_Tab(StorageIO_API, &Info_r.sys_sec) && \
            Storage_Check_Tab(StorageIO_API, &Info_r.user_sec))
        {
            memcpy(p_Info, &Info_r, sizeof(Storage_FlashInfo_TypeDef));
            return true;
        }
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

/* 
 * if matched return data slot address 
 * else return 0
 */
static Storage_Item_TypeDef Storage_Search(Storage_MediumType_List type, Storage_ParaClassType_List class, const char *name)
{
    Storage_Item_TypeDef data_slot;
    StorageIO_TypeDef *StorageIO_API = NULL;
    Storage_BaseSecInfo_TypeDef *p_Sec = NULL;
    Storage_Item_TypeDef *item_list = NULL;
    Storage_Item_TypeDef *p_item = NULL;
    uint32_t tab_addr = 0;
    uint8_t *crc_buf = NULL;
    uint16_t crc_len = 0;
    uint16_t crc = 0;

    memset(&data_slot, 0, sizeof(data_slot));

    if (!Storage_Monitor.init_state || \
        (name == NULL) || \
        (strlen(name) == 0) || \
        (class > Para_User))
        return data_slot;

    switch((uint8_t)type)
    {
        case Internal_Flash:
            StorageIO_API = &InternalFlash_IO;
            p_Sec = Storage_Get_SecInfo(&Storage_Monitor.internal_info, class);
            break;

        case External_Flash:
            StorageIO_API = &ExternalFlash_IO;
            p_Sec = Storage_Get_SecInfo(&Storage_Monitor.external_info, class);
            break;

        default:
            return data_slot;
    }

    if ((p_Sec == NULL) || \
        (p_Sec->para_num == 0) || \
        (p_Sec->para_size == 0))
        return data_slot;

    tab_addr = p_Sec->tab_addr;
    /* tab traverse */
    for (uint8_t tab_i = 0; tab_i < p_Sec->page_num; tab_i ++)
    {
        if (!StorageIO_API->read(tab_addr, page_data_tmp, (p_Sec->tab_size / p_Sec->page_num)))
            return data_slot;
    
        /* tab item traverse */
        item_list = (Storage_Item_TypeDef *)page_data_tmp;
        for (uint16_t item_i = 0; item_i < ((p_Sec->tab_size / p_Sec->page_num) / sizeof(Storage_Item_TypeDef)); item_i ++)
        {
            p_item = &item_list[item_i];

            if ((p_item->head_tag == STORAGE_ITEM_HEAD_TAG) && \
                (p_item->end_tag == STORAGE_ITEM_END_TAG) && \
                (memcmp(p_item->name, name, strlen(name)) == 0))
            {
                if (Storage_Compare_ItemSlot_CRC(*p_item))
                {
                    data_slot = *p_item;
                    return data_slot;
                }
            }
        }
    
        /* update tab address */
        tab_addr += (p_Sec->tab_size / p_Sec->page_num);
    }

    return data_slot;
}

static Storage_ErrorCode_List Storage_SlotData_Update(Storage_MediumType_List type, Storage_ParaClassType_List class, storage_handle data_slot_hdl, uint8_t *p_data, uint16_t size)
{
    StorageIO_TypeDef *StorageIO_API = NULL;
    Storage_BaseSecInfo_TypeDef *p_Sec = NULL;
    Storage_DataSlot_TypeDef *p_slotdata = NULL;
    uint8_t *p_read_tmp = page_data_tmp;
    uint8_t *p_crc = NULL;
    uint16_t crc = 0;
    uint32_t read_addr = 0;
    uint32_t read_size = 0;
    uint32_t update_size = 0;
    uint32_t valid_data_size = 0;
    uint8_t align_byte = 0;

    if (!Storage_Monitor.init_state || \
        (type > External_Flash) || \
        (class > Para_User) || \
        (data_slot_hdl == 0) || \
        (p_data == NULL) || \
        (size == 0))
        return Storage_Param_Error;

    switch((uint8_t)type)
    {
        case External_Flash:
            if (!Storage_Monitor.module_enable_reg.bit.external || \
                !Storage_Monitor.module_init_reg.bit.external)
                return Storage_ModuleInit_Error;

            p_Sec = Storage_Get_SecInfo(&Storage_Monitor.external_info, class);
            StorageIO_API = &ExternalFlash_IO;
            break;

        case Internal_Flash:
            if (!Storage_Monitor.module_enable_reg.bit.internal || \
                !Storage_Monitor.module_init_reg.bit.internal)
                return Storage_ModuleInit_Error;

            p_Sec = Storage_Get_SecInfo(&Storage_Monitor.internal_info, class);
            StorageIO_API = &InternalFlash_IO;
            break;

        default:
            return Storage_Param_Error;
    }

    if ((p_Sec == NULL) || \
        (p_Sec->data_sec_addr > data_slot_hdl) || \
        (data_slot_hdl > (p_Sec->data_sec_addr + p_Sec->data_sec_size)))
        return Storage_Param_Error;

    read_addr = data_slot_hdl;
    memset(page_data_tmp, 0, sizeof(Storage_DataSlot_TypeDef));
    if (size % STORAGE_DATA_ALIGN)
    {
        align_byte = STORAGE_DATA_ALIGN - size % STORAGE_DATA_ALIGN;
    }
    else
        align_byte = 0;

    update_size = size + align_byte;

    /* get data slot first */
    if (!StorageIO_API->read(read_addr, p_read_tmp, sizeof(Storage_DataSlot_TypeDef)))
        return Storage_Read_Error;
    
    /* get data size */
    p_slotdata = (Storage_DataSlot_TypeDef *)p_read_tmp;
    if ((p_slotdata->head_tag != STORAGE_SLOT_HEAD_TAG) || \
        (p_slotdata->total_data_size == 0) || \
        (p_slotdata->total_data_size > p_Sec->data_sec_size))
        return Storage_DataInfo_Error;

    if (p_slotdata->total_data_size != update_size)
        return Storage_Update_DataSize_Error;

    update_size = 0;
    read_size = p_slotdata->total_data_size + sizeof(Storage_DataSlot_TypeDef);
    p_read_tmp = page_data_tmp;
    memset(p_read_tmp, 0, sizeof(Storage_DataSlot_TypeDef));
    p_slotdata = NULL;
    while(true)
    {
        p_read_tmp = page_data_tmp;
        p_slotdata = (Storage_DataSlot_TypeDef *)p_read_tmp;

        /* get data from handle */
        if (!StorageIO_API->read(read_addr, p_read_tmp, read_size))
            return Storage_Read_Error;

        p_slotdata->head_tag = *((uint32_t *)p_read_tmp);
        p_read_tmp += sizeof(p_slotdata->head_tag);
        if (p_slotdata->head_tag != STORAGE_SLOT_HEAD_TAG)
            return Storage_DataInfo_Error;

        memcpy(p_slotdata->name, p_read_tmp, STORAGE_NAME_LEN);
        p_read_tmp += STORAGE_NAME_LEN;

        p_slotdata->total_data_size = *((uint32_t *)p_read_tmp);
        p_read_tmp += sizeof(p_slotdata->total_data_size);
        if ((p_slotdata->total_data_size == 0) || \
            (p_slotdata->total_data_size > p_Sec->data_sec_size))
            return Storage_DataInfo_Error;

        p_slotdata->cur_slot_size = *((uint32_t *)p_read_tmp);
        p_read_tmp += sizeof(p_slotdata->cur_slot_size);
        if (p_slotdata->cur_slot_size > p_slotdata->total_data_size)
            return Storage_DataInfo_Error;

        p_slotdata->nxt_addr = *((uint32_t *)p_read_tmp);
        p_read_tmp += sizeof(p_slotdata->nxt_addr);
        if (p_slotdata->nxt_addr)
        {
            if ((p_slotdata->nxt_addr < p_Sec->data_sec_addr) || \
                (p_slotdata->nxt_addr > p_Sec->data_sec_addr + p_Sec->data_sec_size))
                return Storage_DataInfo_Error;
        }

        p_slotdata->align_size = *((uint8_t *)p_read_tmp);
        p_read_tmp += sizeof(p_slotdata->align_size);
        if (p_slotdata->align_size >= STORAGE_DATA_ALIGN)
            return Storage_DataInfo_Error;

        memcpy(p_read_tmp, p_data, (p_slotdata->cur_slot_size - p_slotdata->align_size));
        p_read_tmp += p_slotdata->cur_slot_size - p_slotdata->align_size;

        if (p_slotdata->align_size)
        {
            /* set align byte */
            memset(p_read_tmp, 0, p_slotdata->align_size);
            p_read_tmp += p_slotdata->align_size;
        }

        valid_data_size += p_slotdata->cur_slot_size - p_slotdata->align_size;
        if (p_slotdata->nxt_addr == 0)
        {
            /* all data has been read out from the data slot in destination section */
            /* compare update data size and valid_data_size */
            if (size != valid_data_size)
                return Storage_Update_DataSize_Error;
        }

        /* update crc */
        crc = Common_CRC16(p_data, p_slotdata->cur_slot_size) ;
        memcpy(p_read_tmp, &crc, sizeof(crc));
        
        p_data += p_slotdata->cur_slot_size - p_slotdata->align_size;
        p_read_tmp += sizeof(p_slotdata->slot_crc);

        if (*(uint32_t *)p_read_tmp != STORAGE_SLOT_END_TAG)
            return Storage_DataInfo_Error;

        if ((p_slotdata->nxt_addr == 0) && \
            (size == valid_data_size))
        {
            /* update data to flash */
            if (!StorageIO_API->write(read_addr, page_data_tmp, p_slotdata->cur_slot_size + sizeof(Storage_DataSlot_TypeDef)))
                return Storage_Write_Error;

            /* update accomplish */
            return Storage_Error_None;
        }

        read_addr = p_slotdata->nxt_addr;
    }

    return Storage_Error_None;
}

/* developping */
static bool Storage_Link_FreeSlot(uint32_t new_free_slot, Storage_BaseSecInfo_TypeDef *p_Sec, StorageIO_TypeDef *StorageIO_API)
{
    if ((new_free_slot == 0) || \
        (p_Sec == NULL) || \
        (StorageIO_API == NULL))
        return false;

    return false;
}

/* developping */
static Storage_ErrorCode_List Storage_FreeSlot_CheckMerge(uint32_t slot_addr, Storage_FreeSlot_TypeDef *slot_info, Storage_BaseSecInfo_TypeDef *p_Sec, StorageIO_TypeDef *StorageIO_API)
{
    Storage_FreeSlot_TypeDef FreeSlot_Info;
    uint32_t front_freeslot_addr = 0;
    uint32_t behand_freeslot_addr = 0;
    uint32_t ori_freespace_size = 0;

    if ((p_Sec == NULL) || \
        (slot_info == NULL) || \
        (slot_addr < p_Sec->data_sec_addr) || \
        (slot_addr > (p_Sec->data_sec_addr + p_Sec->data_sec_size)) || \
        (StorageIO_API == NULL))
        return Storage_Param_Error;

    memset(&FreeSlot_Info, 0, sizeof(FreeSlot_Info));

    if ((slot_info->head_tag != STORAGE_SLOT_HEAD_TAG) || \
        (slot_info->end_tag != STORAGE_SLOT_END_TAG) || \
        (slot_info->cur_slot_size > p_Sec->free_space_size))
        return Storage_FreeSlot_Info_Error;

    ori_freespace_size = p_Sec->free_space_size;
    front_freeslot_addr = p_Sec->free_slot_addr;
    while (true)
    {
        /* traverse all free slot */
        if (!StorageIO_API->read(front_freeslot_addr, &FreeSlot_Info, sizeof(FreeSlot_Info)))
            return Storage_Read_Error;

        if ((FreeSlot_Info.head_tag != STORAGE_SLOT_HEAD_TAG) || \
            (FreeSlot_Info.end_tag != STORAGE_SLOT_END_TAG))
            return Storage_FreeSlot_Info_Error;

        p_Sec->free_space_size += slot_info->cur_slot_size;

        /* circumstance 1: new free slot in front of the old free slot */
        if (slot_addr + slot_info->cur_slot_size == front_freeslot_addr)
        {
            /* merge front free slot */
            if (slot_info->nxt_addr != front_freeslot_addr)
            {
                /* free space is uncontiguous */
                /* error occur */
                Storage_Assert(true);
            }

            slot_info->nxt_addr = FreeSlot_Info.nxt_addr;
            slot_info->cur_slot_size += FreeSlot_Info.cur_slot_size + sizeof(Storage_FreeSlot_TypeDef);

            memset(&FreeSlot_Info, 0, sizeof(FreeSlot_Info));

            /* write to histort freeslot address */
            if (!StorageIO_API->write(front_freeslot_addr, &FreeSlot_Info, sizeof(FreeSlot_Info)))
            {
                p_Sec->free_space_size = ori_freespace_size;
                return Storage_Write_Error;
            }

            /* write to current freeslot section */
            if (!StorageIO_API->write(slot_addr, slot_info, sizeof(Storage_FreeSlot_TypeDef)))
            {
                p_Sec->free_space_size = ori_freespace_size;
                return Storage_Write_Error;
            }

            p_Sec->free_slot_addr = slot_addr;
            p_Sec->free_space_size += sizeof(Storage_FreeSlot_TypeDef);
        }
        /* circumstance 2: new free slot is behand of the old free slot */
        else if (front_freeslot_addr + FreeSlot_Info.cur_slot_size == slot_addr)
        {
            /* merge behand free slot */
            FreeSlot_Info.cur_slot_size += slot_info->cur_slot_size + sizeof(Storage_FreeSlot_TypeDef);
        }
        else
        {
            /* link free slot */
            // if (Storage_Link_FreeSlot(uint32_t new_free_slot, p_Sec, StorageIO_API))
                // return Storage_Error_None;
        }

        if (FreeSlot_Info.nxt_addr == 0)
            return Storage_Error_None;

        /* update front free slot address */
        front_freeslot_addr = FreeSlot_Info.nxt_addr;
    }

    return Storage_Delete_Error;
}

/* developping */
static bool Storage_DeleteSingalDataSlot(uint32_t slot_addr, uint8_t *p_data, Storage_BaseSecInfo_TypeDef *p_Sec, StorageIO_TypeDef *StorageIO_API)
{
    uint32_t cur_slot_size = 0;
    uint32_t inc_free_space = sizeof(Storage_DataSlot_TypeDef);
    uint8_t *p_freeslot_start = NULL;
    uint8_t *data_w = NULL;

    if ((slot_addr == 0) || \
        (StorageIO_API == NULL) || \
        (p_Sec == NULL) || \
        (p_data == NULL) || \
        (slot_addr < p_Sec->data_sec_addr) || \
        (slot_addr > (p_Sec->data_sec_addr + p_Sec->data_sec_size)))
        return false;
    
    /* check header */
    if (*((uint32_t *)p_data) != STORAGE_SLOT_HEAD_TAG)
        return false;

    p_freeslot_start = p_data;
    data_w = p_freeslot_start;
    p_data += sizeof(uint32_t);

    /* clear current slot name */
    memset(p_data, 0, STORAGE_NAME_LEN);
    p_data += STORAGE_NAME_LEN;

    /* clear total data size */
    *((uint32_t *)p_data) = 0;
    p_data += sizeof(uint32_t);

    /* get current slot data size */
    cur_slot_size = *((uint32_t *)p_data);
    inc_free_space += cur_slot_size;
    
    /* clear current slot data size */
    *((uint32_t *)p_data) = 0;
    p_data += sizeof(uint32_t);

    /* clear next data slot address */
    *((uint32_t *)p_data) = 0;
    p_data += sizeof(uint32_t);

    /* clear align size */
    *((uint8_t *)p_data) = 0;
    p_data += sizeof(uint8_t);

    /* clear data */
    memset(p_data, 0, cur_slot_size);
    p_data += cur_slot_size;

    /* clear crc */
    *((uint16_t *)p_data) = 0;
    p_data += sizeof(uint16_t);

    /* clear end tag */
    *((uint32_t *)p_data) = 0;

    /* update freeslot data info */
    if (*(uint32_t *)p_freeslot_start == STORAGE_SLOT_HEAD_TAG)
    {
        p_freeslot_start += sizeof(uint32_t);

        /* update current free slot size */
        *(uint32_t *)p_freeslot_start = cur_slot_size;
        p_freeslot_start += sizeof(uint32_t);

        /* reset next freeslot addr
         * link free slot address
         *
         * should link free slot
         * not set next free slot address only
         * still bug */
        *(uint32_t *)p_freeslot_start = p_Sec->free_slot_addr;
        p_freeslot_start += sizeof(uint32_t);

        /* set end tag */
        *(uint32_t *)p_freeslot_start = STORAGE_SLOT_END_TAG;
    }
    else
        return false;

    /* update to data section */
    if (StorageIO_API->write(slot_addr, data_w, inc_free_space))
    {
        /* check free slot and merge */
        if (Storage_FreeSlot_CheckMerge(slot_addr, (Storage_FreeSlot_TypeDef *)p_freeslot_start, p_Sec, StorageIO_API) == Storage_Error_None)
            return true;
    }

    return false;
}

/* developping */
static bool Storage_DeleteAllDataSlot(uint32_t addr, char *name, uint32_t total_size, Storage_BaseSecInfo_TypeDef *p_Sec, StorageIO_TypeDef *StorageIO_API)
{
    Storage_DataSlot_TypeDef data_slot;
    uint8_t *p_read = page_data_tmp;
    uint8_t name_len = 0;
    uint32_t delete_size = 0;
    uint32_t cur_freeslot_addr = 0;

    if ((addr == 0) || \
        (name == NULL) || \
        (strlen(name) == 0) || \
        (total_size == 0) || \
        (p_Sec == NULL) || \
        (StorageIO_API == NULL))
        return false;

    memset(&data_slot, 0, sizeof(data_slot));
    name_len = strlen(name);
    
    if (!StorageIO_API->read(addr, page_data_tmp, total_size))
        return false;

    data_slot.head_tag = *((uint32_t *)p_read);
    if (data_slot.head_tag != STORAGE_SLOT_HEAD_TAG)
        return false;

    p_read += sizeof(data_slot.head_tag);
    memcpy(data_slot.name, p_read, STORAGE_NAME_LEN);
    if (memcmp(data_slot.name, name, name_len) != 0)
        return false;

    p_read += STORAGE_NAME_LEN;
    data_slot.total_data_size = *((uint32_t *)p_read);
    if (data_slot.total_data_size != total_size)
        return false;

    p_read += sizeof(data_slot.total_data_size);
    data_slot.cur_slot_size = *((uint32_t *)p_read);
    if (data_slot.cur_slot_size > data_slot.total_data_size)
        return false;

    delete_size = data_slot.cur_slot_size;
    p_read += sizeof(data_slot.cur_slot_size);
    data_slot.nxt_addr = *((uint32_t *)p_read);
    if ((data_slot.nxt_addr < p_Sec->data_sec_addr) || \
        (data_slot.nxt_addr > (p_Sec->data_sec_addr + p_Sec->data_sec_size)))
        return false;
    
    p_read += sizeof(data_slot.nxt_addr);
    data_slot.align_size = *((uint8_t *)p_read);    
    if (data_slot.align_size >= STORAGE_DATA_ALIGN)
        return false;

    /* set current slot data as 0 */
    p_read += sizeof(data_slot.align_size);
    memset(p_read, 0, data_slot.cur_slot_size);
    p_read += data_slot.cur_slot_size;

    /* clear crc */
    *((uint16_t *)p_read) = 0;
    p_read += sizeof(data_slot.slot_crc);

    /* ender error */
    if (*((uint32_t *)p_read) != STORAGE_SLOT_END_TAG)
        return false;

    /* delete next slot data */
    if (data_slot.nxt_addr)
    {
        /* traverse slot address */
        if (!Storage_DeleteAllDataSlot(data_slot.nxt_addr, name, total_size, p_Sec, StorageIO_API))
            return false;
    }

    /* reset data slot as free slot */
    if (!Storage_DeleteSingalDataSlot(addr, page_data_tmp, p_Sec, StorageIO_API))
        return false;

    /* update section info free address */

    /* update base info free address */

    return true;
}

/* developping */
static Storage_ErrorCode_List Storage_DeleteItem(Storage_MediumType_List type, Storage_ParaClassType_List class, const char *name, uint32_t size)
{
    Storage_FlashInfo_TypeDef *p_Flash = NULL;
    Storage_BaseSecInfo_TypeDef *p_Sec = NULL;
    Storage_Item_TypeDef *item_list = NULL;
    StorageIO_TypeDef *StorageIO_API = NULL;
    Storage_Item_TypeDef Item;
    
    memset(&Item, 0, sizeof(Item));

    if( !Storage_Monitor.init_state || \
        (name == NULL) || \
        (strlen(name) == 0) || \
        (strlen(name) >= STORAGE_NAME_LEN) || \
        (size == 0))
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
    if ((p_Sec == NULL) || \
        (p_Sec->para_num == 0) || \
        (p_Sec->para_size == 0))
        return Storage_Class_Error;

    if ((StorageIO_API->erase == NULL) || \
        (StorageIO_API->read == NULL) || \
        (StorageIO_API->write == NULL))
        return Storage_ModuleAPI_Error;

    /* search tab for item slot first */
    Item = Storage_Search(type, class, name);
    if ((Item.data_addr) && \
        (Item.head_tag == STORAGE_ITEM_HEAD_TAG) && \
        (Item.end_tag == STORAGE_ITEM_END_TAG))
    {
        /* Item found */
        if (Storage_DeleteDataSlot(Item.data_addr, name, Item.len, p_Sec, StorageIO_API))
            return Storage_Error_None;
    }

    /* update tab */

    return Storage_Delete_Error;
}

static Storage_ErrorCode_List Storage_CreateItem(Storage_MediumType_List type, Storage_ParaClassType_List class, const char *name, uint8_t *p_data, uint32_t size)
{
    uint8_t *crc_buf = NULL;
    uint8_t *slot_update_ptr = NULL;
    uint16_t base_info_crc = 0;
    uint32_t storage_data_size = 0;
    uint32_t stored_size = 0;
    uint32_t store_addr = 0;
    uint32_t unstored_size = 0;
    uint32_t storage_tab_addr = 0;
    uint32_t cur_freeslot_addr = 0;
    uint32_t nxt_freeslot_addr = 0;
    uint32_t slot_useful_size = 0;
    uint8_t item_index = 0;
    StorageIO_TypeDef *StorageIO_API = NULL;
    Storage_FlashInfo_TypeDef *p_Flash = NULL;
    Storage_BaseSecInfo_TypeDef *p_Sec = NULL;
    Storage_Item_TypeDef *tab_item = NULL;
    Storage_Item_TypeDef crt_item_slot;
    Storage_FreeSlot_TypeDef FreeSlot;
    Storage_DataSlot_TypeDef DataSlot;
    uint8_t align_byte = 0;

    memset(&crt_item_slot, 0, sizeof(crt_item_slot));
    memset(&DataSlot, 0, sizeof(Storage_DataSlot_TypeDef));
    memset(&FreeSlot, 0, sizeof(Storage_FreeSlot_TypeDef));

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

    if (p_Sec->free_slot_addr == 0)
        return Storage_FreeSlot_Addr_Error;

    if (StorageIO_API->read(p_Sec->free_slot_addr, &FreeSlot, sizeof(Storage_FreeSlot_TypeDef)) && \
        (FreeSlot.head_tag == STORAGE_SLOT_HEAD_TAG) && \
        (FreeSlot.end_tag == STORAGE_SLOT_END_TAG) && \
        (strlen(name) <= STORAGE_NAME_LEN))
    {
        cur_freeslot_addr = p_Sec->free_slot_addr;

        if (size % STORAGE_DATA_ALIGN)
        {
            align_byte = STORAGE_DATA_ALIGN - size % STORAGE_DATA_ALIGN;
        }
        else
            align_byte = 0;

        p_Sec->para_num ++;
        p_Sec->para_size += (size + align_byte);
        
        storage_tab_addr = p_Sec->tab_addr;
        for(uint16_t tab_i = 0; tab_i < p_Sec->page_num; tab_i ++)
        {
            store_addr = 0;
            item_index = 0;

            /* step 1: search tab */
            if (!StorageIO_API->read(storage_tab_addr, page_data_tmp, (p_Sec->tab_size / p_Sec->page_num)))
                return Storage_Read_Error;

            tab_item = (Storage_Item_TypeDef *)page_data_tmp;
            for (uint16_t item_i = 0; item_i < Item_Capacity_Per_Tab; item_i ++)
            {
                if (((tab_item[item_i].head_tag == STORAGE_ITEM_HEAD_TAG) && \
                     (tab_item[item_i].end_tag == STORAGE_ITEM_END_TAG) && \
                     (memcmp(tab_item[item_i].name, STORAGE_FREEITEM_NAME, strlen(STORAGE_FREEITEM_NAME)) == 0)) || \
                    ((tab_item[item_i].head_tag != STORAGE_ITEM_HEAD_TAG) && \
                     (tab_item[item_i].end_tag != STORAGE_ITEM_END_TAG)))
                {
                    item_index = item_i;

                    /* found empty item slot */
                    crt_item_slot = tab_item[item_i];

                    /* set item slot info */
                    crt_item_slot.class = class;
                    memset(crt_item_slot.name, '\0', STORAGE_NAME_LEN);
                    memcpy(crt_item_slot.name, name, strlen(name));
                    crt_item_slot.len = size + align_byte;

                    /* set free slot address as current data address */
                    crt_item_slot.data_addr = p_Sec->free_slot_addr;
                    crt_item_slot.head_tag = STORAGE_ITEM_HEAD_TAG;
                    crt_item_slot.end_tag = STORAGE_ITEM_END_TAG;

                    /* comput crc */
                    Storage_Comput_ItemSlot_CRC(&crt_item_slot);
                    store_addr = crt_item_slot.data_addr;
                    break; 
                }
            }

            if (store_addr)
                break;

            storage_tab_addr += (p_Sec->tab_size / p_Sec->page_num);
        }

        if (store_addr == 0)
            return Storage_DataAddr_Update_Error;

        storage_data_size = size;
        /* get align byte size */
        /* noticed: write 0 on align space */
        storage_data_size += align_byte;

        /* noticed: DataSlot.cur_data_size - DataSlot.align_size is current slot storaged data size */
        DataSlot.total_data_size = storage_data_size;
        unstored_size = DataSlot.total_data_size;

        if (p_Sec->free_space_size >= storage_data_size)
        {
            while(true)
            {
                /* step 2: comput storage data size and set data slot */
                DataSlot.head_tag = STORAGE_SLOT_HEAD_TAG;
                DataSlot.end_tag = STORAGE_SLOT_END_TAG;
                memset(DataSlot.name, '\0', STORAGE_NAME_LEN);
                memcpy(DataSlot.name, name, strlen(name));
                
                if (FreeSlot.cur_slot_size <= sizeof(Storage_DataSlot_TypeDef))
                    return Storage_No_Enough_Space;

                crc_buf = p_data + stored_size;
                slot_useful_size = FreeSlot.cur_slot_size - sizeof(Storage_DataSlot_TypeDef);
                /* current have space for new data need to be storage */
                if (slot_useful_size < storage_data_size)
                {
                    /*
                        * UNTESTED IN THIS BRANCH
                        */
                    
                    DataSlot.cur_slot_size = slot_useful_size;
                    stored_size += DataSlot.cur_slot_size;
                    unstored_size -= stored_size;
                    DataSlot.align_size = 0;
                    
                    /* current free slot full fill can not split any space for next free slot`s start */
                    DataSlot.nxt_addr = FreeSlot.nxt_addr;

                    /* in light of current free slot not enough for storage data, 
                        * then find next free slot used for storage data remaining */
                    cur_freeslot_addr = FreeSlot.nxt_addr;
                    if (!StorageIO_API->read(cur_freeslot_addr, &FreeSlot, sizeof(Storage_FreeSlot_TypeDef)))
                        return Storage_FreeSlot_Get_Error;   
                }
                else
                {
                    /* seperate data slot and new free slot from current free slot */
                    stored_size += unstored_size;
                    DataSlot.cur_slot_size = unstored_size;
                    DataSlot.align_size = align_byte;
                    DataSlot.nxt_addr = 0;

                    /* update current free slot adderess */
                    cur_freeslot_addr += DataSlot.cur_slot_size + sizeof(Storage_DataSlot_TypeDef);
                    FreeSlot.cur_slot_size -= DataSlot.cur_slot_size + sizeof(Storage_DataSlot_TypeDef);
                }

                p_Sec->free_space_size -= DataSlot.cur_slot_size;

                /* comput current slot crc */
                DataSlot.slot_crc = Common_CRC16(crc_buf, DataSlot.cur_slot_size);

                /* write to the data section */
                /* storage target data */
                slot_update_ptr = page_data_tmp;
                memcpy(slot_update_ptr, &DataSlot.head_tag, sizeof(DataSlot.head_tag));
                slot_update_ptr += sizeof(DataSlot.head_tag);
                memcpy(slot_update_ptr, DataSlot.name, STORAGE_NAME_LEN);
                slot_update_ptr += STORAGE_NAME_LEN;
                memcpy(slot_update_ptr, &DataSlot.total_data_size, sizeof(DataSlot.total_data_size));
                slot_update_ptr += sizeof(DataSlot.total_data_size);
                memcpy(slot_update_ptr, &DataSlot.cur_slot_size, sizeof(DataSlot.cur_slot_size));
                slot_update_ptr += sizeof(DataSlot.cur_slot_size);
                memcpy(slot_update_ptr, &DataSlot.nxt_addr, sizeof(DataSlot.nxt_addr));
                slot_update_ptr += sizeof(DataSlot.nxt_addr);
                memcpy(slot_update_ptr, &DataSlot.align_size, sizeof(DataSlot.align_size));
                slot_update_ptr += sizeof(DataSlot.align_size);
                memcpy(slot_update_ptr, crc_buf, (DataSlot.cur_slot_size - DataSlot.align_size));
                slot_update_ptr += (DataSlot.cur_slot_size - DataSlot.align_size);
                
                if (DataSlot.align_size)
                {
                    memset(slot_update_ptr, 0, DataSlot.align_size);
                    slot_update_ptr += DataSlot.align_size;
                }

                memcpy(slot_update_ptr, &DataSlot.slot_crc, sizeof(DataSlot.slot_crc));
                slot_update_ptr += sizeof(DataSlot.slot_crc);
                memcpy(slot_update_ptr, &DataSlot.end_tag, sizeof(DataSlot.end_tag));
                slot_update_ptr += sizeof(DataSlot.end_tag);

                /* step 3: store data to data section */
                if (!StorageIO_API->write(store_addr, page_data_tmp, (DataSlot.cur_slot_size + sizeof(DataSlot))))
                    return Storage_Write_Error;

                if (DataSlot.nxt_addr == 0)
                {
                    Storage_Assert(DataSlot.total_data_size > stored_size);
                    if (DataSlot.total_data_size == stored_size)
                    {
                        /* step 4: update free slot */
                        if (!StorageIO_API->write(cur_freeslot_addr, &FreeSlot, sizeof(FreeSlot)))
                            return Storage_Write_Error;

                        break;
                    }

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
        }

        /* get tab */
        if (!StorageIO_API->read(storage_tab_addr, page_data_tmp, (p_Sec->tab_size / p_Sec->page_num)))
            return Storage_Read_Error;

        tab_item = (Storage_Item_TypeDef *)page_data_tmp;
        tab_item[item_index] = crt_item_slot;

        /* write back item slot list to tab */
        if (!StorageIO_API->write(storage_tab_addr, page_data_tmp, (p_Sec->tab_size / p_Sec->page_num)))
            return Storage_Write_Error;

        /* update free slot address in base info */
        p_Sec->free_slot_addr = cur_freeslot_addr;

        /* update base info crc */
        memcpy(page_data_tmp, p_Flash, sizeof(Storage_FlashInfo_TypeDef));
        base_info_crc = Common_CRC16(page_data_tmp, Storage_InfoPageSize - sizeof(base_info_crc));
        memcpy(&page_data_tmp[Storage_InfoPageSize - sizeof(base_info_crc)], &base_info_crc, sizeof(base_info_crc));
        
        /* update base info from section start*/
        if (!StorageIO_API->write(From_Start_Address, page_data_tmp, Storage_InfoPageSize))
            return Storage_Write_Error;
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
        free_slot.cur_slot_size = p_SecInfo->data_sec_size - sizeof(free_slot);
        free_slot.nxt_addr = 0;
        /* if current free slot get enought space for data then set next address (nxt_addr) as 0 */
        /* or else setnext address (nxt_addr) as next slot address such as 0x80exxxx. */
        /* until all data was saved into multiple flash segment completely */
        /* set ender tag as 0xFE1001FE */
        free_slot.end_tag = STORAGE_SLOT_END_TAG;
        
        memcpy(page_data_tmp, &free_slot, sizeof(free_slot));
        if (!StorageIO_API->write(p_SecInfo->data_sec_addr, page_data_tmp, Storage_TabSize))
            return false;

        /* update info section */
        p_SecInfo->free_slot_addr = p_SecInfo->data_sec_addr;
        p_SecInfo->free_space_size = p_SecInfo->data_sec_size;
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
    uint32_t sector_size = 0;

    memset(&Info, 0, sizeof(Storage_FlashInfo_TypeDef));
    memset(&Info_Rx, 0, sizeof(Storage_FlashInfo_TypeDef));
    
    switch((uint8_t)type)
    {
        case Internal_Flash:
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
            
            Info.boot_sec.tab_addr = BaseInfo_start_addr + Storage_InfoPageSize;
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
            
            Info.boot_sec.tab_addr = Storage_InfoPageSize;
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
            if(Info.total_size < tab_addr_offset)
                return false;
            
            remain_data_sec_size = Info.total_size - tab_addr_offset;
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
    memset(page_data_tmp, 0, Storage_InfoPageSize);

    /* read out and erase sector */
    if (!StorageIO_API->read(addr_offset, page_data_tmp, sizeof(Info)))
        return false;

    /* write base info to info section */
    memcpy(page_data_tmp, &Info, sizeof(Info));
    crc = Common_CRC16(page_data_tmp, Storage_InfoPageSize - sizeof(crc));
    memcpy(&page_data_tmp[Storage_InfoPageSize - sizeof(crc)], &crc, sizeof(crc));

    /* write into flash chip */
    if (!StorageIO_API->write(addr_offset, page_data_tmp, Storage_InfoPageSize))
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

static bool Storage_Compare_ItemSlot_CRC(const Storage_Item_TypeDef item)
{
    uint8_t *crc_buf = (uint8_t *)&item;
    uint16_t crc_len = sizeof(Storage_Item_TypeDef);
    uint16_t crc = 0;
    
    crc_buf += sizeof(item.head_tag);
    crc_len -= sizeof(item.head_tag);
    crc_len -= sizeof(item.end_tag);
    crc_len -= sizeof(item.crc16);
    crc = Common_CRC16(crc_buf, crc_len);

    if (crc == item.crc16)
        return true;

    return false;
}

static bool Storage_Comput_ItemSlot_CRC(Storage_Item_TypeDef *p_item)
{
    uint8_t *crc_buf = NULL;
    uint16_t crc_len = sizeof(Storage_Item_TypeDef);
    
    if (p_item)
    {
        crc_buf = ((uint8_t *)p_item) + sizeof(p_item->head_tag);
        crc_len -= sizeof(p_item->head_tag);
        crc_len -= sizeof(p_item->end_tag);
        crc_len -= sizeof(p_item->crc16);
        p_item->crc16 = Common_CRC16(crc_buf, crc_len);

        return true;
    }

    return false;
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
    uint32_t read_start_addr = 0;
    uint32_t flash_end_addr = 0;
    uint32_t section_start_addr = 0;
    uint32_t next_read_addr = 0;
    uint32_t section_size = 0;
    uint32_t read_offset = 0;
    uint32_t read_len = len;
    Storage_ExtFLashDevObj_TypeDef *dev = NULL;
    DevW25Qxx_Error_List state = DevW25Qxx_Ok;

    if ((Storage_Monitor.ExtDev_ptr != NULL) && p_data && len)
    {
        read_start_addr = Storage_Monitor.external_info.base_addr + addr_offset;       
        dev = (Storage_ExtFLashDevObj_TypeDef *)(Storage_Monitor.ExtDev_ptr);

        switch((uint8_t)dev->chip_type)
        {
            case Storage_ChipType_W25Qxx:
                if (dev->dev_api && dev->dev_obj)
                {
                    section_size = To_DevW25Qxx_API(dev->dev_api)->info(To_DevW25Qxx_OBJ(dev->dev_obj)).subsector_size;
                    /* get w25qxx device info */
                    /* address check */
                    flash_end_addr = To_DevW25Qxx_API(dev->dev_api)->info(To_DevW25Qxx_OBJ(dev->dev_obj)).start_addr;
                    if (flash_end_addr > read_start_addr)
                        return false;

                    /* range check */
                    flash_end_addr += To_DevW25Qxx_API(dev->dev_api)->info(To_DevW25Qxx_OBJ(dev->dev_obj)).flash_size;
                    if ((len + read_start_addr) > flash_end_addr)
                        return false;

                    if (section_size)
                    {
                        section_start_addr = To_DevW25Qxx_API(dev->dev_api)->get_section_start_addr(To_DevW25Qxx_OBJ(dev->dev_obj), read_start_addr);
                        read_offset = read_start_addr - section_start_addr;
                        if (section_size > sizeof(flash_read_tmp))
                            return false;

                        while(true)
                        {
                            /* circumstances 1: store data size less than flash sector size and only none multiple sector read is needed */
                            /* circumstances 2: store data size less than flash sector length but need to read from the end of the sector N to the start of the sector N + 1 */
                            /* circumstances 3: store data size large than flash sector length */
                            if (read_offset + read_len > section_size)
                                read_len = section_size - read_offset;

                            /* read whole section */
                            if (To_DevW25Qxx_API(dev->dev_api)->read(To_DevW25Qxx_OBJ(dev->dev_obj), section_start_addr, flash_read_tmp, section_size) != DevW25Qxx_Ok)
                                return false;
                        
                            memcpy(p_data, flash_read_tmp + read_offset, read_len);
                            memset(flash_read_tmp, 0, section_size);

                            len -= read_len;
                            if (len == 0)
                                return true;
                        
                            read_offset = 0;
                            next_read_addr = To_DevW25Qxx_API(dev->dev_api)->get_section_start_addr(To_DevW25Qxx_OBJ(dev->dev_obj), section_start_addr + read_len);
                            if (next_read_addr == section_start_addr)
                                read_offset = read_len;
                            
                            p_data += read_len;
                            read_len = len;
                            section_start_addr = next_read_addr;
                        }
                    }
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
    uint32_t write_start_addr = 0;
    uint32_t flash_end_addr = 0;
    uint32_t section_start_addr = 0;
    uint32_t next_write_addr = 0;
    uint32_t section_size = 0;
    uint32_t write_offset = 0;
    uint32_t write_len = len;
    Storage_ExtFLashDevObj_TypeDef *dev = NULL;
    DevW25Qxx_Error_List state = DevW25Qxx_Ok;

    if ((Storage_Monitor.ExtDev_ptr != NULL) && p_data && len)
    {
        write_start_addr = Storage_Monitor.external_info.base_addr + addr_offset;
        dev = (Storage_ExtFLashDevObj_TypeDef *)(Storage_Monitor.ExtDev_ptr);
    
        switch((uint8_t)dev->chip_type)
        {
            case Storage_ChipType_W25Qxx:
                if (dev->dev_api && dev->dev_obj)
                {
                    section_size = To_DevW25Qxx_API(dev->dev_api)->info(To_DevW25Qxx_OBJ(dev->dev_obj)).subsector_size;
                    /* get w25qxx device info */
                    /* address check */
                    flash_end_addr = To_DevW25Qxx_API(dev->dev_api)->info(To_DevW25Qxx_OBJ(dev->dev_obj)).start_addr;
                    if (flash_end_addr > write_start_addr)
                        return false;

                    /* range check */
                    flash_end_addr += To_DevW25Qxx_API(dev->dev_api)->info(To_DevW25Qxx_OBJ(dev->dev_obj)).flash_size;
                    if ((len + write_start_addr) > flash_end_addr)
                        return false;
                    
                    if (section_size)
                    {
                        section_start_addr = To_DevW25Qxx_API(dev->dev_api)->get_section_start_addr(To_DevW25Qxx_OBJ(dev->dev_obj), write_start_addr);
                        write_offset = write_start_addr - section_start_addr;
                        if (section_size > sizeof(flash_write_tmp))
                            return false;

                        while(true)
                        {
                            /* circumstances 1: store data size less than flash sector size and only none multiple sector write is needed */
                            /* circumstances 2: store data size less than flash sector length but need to write from the end of the sector N to the start of the sector N + 1 */
                            /* circumstances 3: store data size large than flash sector length */
                            /* read whole section */
                            if (To_DevW25Qxx_API(dev->dev_api)->read(To_DevW25Qxx_OBJ(dev->dev_obj), section_start_addr, flash_write_tmp, section_size) != DevW25Qxx_Ok)
                                return false;

                            /* erase whole section */
                            if (To_DevW25Qxx_API(dev->dev_api)->erase_sector(To_DevW25Qxx_OBJ(dev->dev_obj), section_start_addr) != DevW25Qxx_Ok)
                                return false;

                            /* update whole section */
                            if (write_offset + write_len > section_size)
                                write_len = section_size - write_offset;
                            
                            /* copy data to section data read out */
                            memcpy(flash_write_tmp + write_offset, p_data, write_len);

                            state = To_DevW25Qxx_API(dev->dev_api)->write(To_DevW25Qxx_OBJ(dev->dev_obj), section_start_addr, flash_write_tmp, section_size);

                            /* clear cache buff */
                            memset(flash_write_tmp, 0, section_size);
                            
                            len -= write_len;
                            if (state == DevW25Qxx_Ok)
                            {
                                if (len == 0)
                                    return true;
                            }
                            else
                                return false;

                            write_offset = 0;
                            next_write_addr = To_DevW25Qxx_API(dev->dev_api)->get_section_start_addr(To_DevW25Qxx_OBJ(dev->dev_obj), section_start_addr + write_len);
                            if (next_write_addr == section_start_addr)
                                write_offset = write_len;

                            p_data += write_len;
                            write_len = len;
                            section_start_addr = next_write_addr; 
                        }
                    }
                }
                return false;

            default:
                return false;
        }
    }
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
    shellPrint(obj, "\t\tfree slot size:  %x\r\n", p_SecInfo->free_space_size);
    shellPrint(obj, "\t\tparam num:       %d\r\n", p_SecInfo->para_num);
    shellPrint(obj, "\t\tparam size:      %d\r\n", p_SecInfo->para_size);
}

static const char* Storage_Error_Print(Storage_ErrorCode_List code)
{
    switch((uint8_t) code)
    {
        case Storage_Error_None:
            return Storage_ErrorCode_ToStr(Storage_Error_None);
        
        case Storage_BusType_Error:
            return Storage_ErrorCode_ToStr(Storage_BusType_Error);

        case Storage_BusCfg_Malloc_Failed:
            return Storage_ErrorCode_ToStr(Storage_BusCfg_Malloc_Failed);

        case Storage_Param_Error:
            return Storage_ErrorCode_ToStr(Storage_Param_Error);

        case Storage_ExtDevObj_Error:
            return Storage_ErrorCode_ToStr(Storage_ExtDevObj_Error);

        case Storage_ModuleType_Error:
            return Storage_ErrorCode_ToStr(Storage_ModuleType_Error);

        case Storage_ModuleInit_Error:
            return Storage_ErrorCode_ToStr(Storage_ModuleInit_Error);

        case Storage_BusInit_Error:
            return Storage_ErrorCode_ToStr(Storage_BusInit_Error);

        case Storage_Read_Error:
            return Storage_ErrorCode_ToStr(Storage_Read_Error);

        case Storage_Write_Error:
            return Storage_ErrorCode_ToStr(Storage_Write_Error);

        case Storage_Erase_Error:
            return Storage_ErrorCode_ToStr(Storage_Erase_Error);

        case Storage_NameMatched:
            return Storage_ErrorCode_ToStr(Storage_NameMatched);

        case Storage_ModuleAPI_Error:
            return Storage_ErrorCode_ToStr(Storage_ModuleAPI_Error);

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

        case Storage_FreeSlot_Get_Error:
            return Storage_ErrorCode_ToStr(Storage_FreeSlot_Get_Error);

        case Storage_DataAddr_Update_Error:
            return Storage_ErrorCode_ToStr(Storage_DataAddr_Update_Error);

        case Storage_No_Enough_Space:
            return Storage_ErrorCode_ToStr(Storage_No_Enough_Space);
        
        case Storage_FreeSlot_Addr_Error:
            return Storage_ErrorCode_ToStr(Storage_FreeSlot_Addr_Error);
        
        case Storage_ItemInfo_Error:
            return Storage_ErrorCode_ToStr(Storage_ItemInfo_Error);
            
        case Storage_CRC_Error:
            return Storage_ErrorCode_ToStr(Storage_CRC_Error);
            
        case Storage_Update_DataSize_Error:
            return Storage_ErrorCode_ToStr(Storage_Update_DataSize_Error);
            
        default:
            return "Unknow Error\r\n";
    }
}

static bool Storage_Get_Flash_Section_IOAPI(Shell *shell_obj, \
                                            Storage_MediumType_List medium, \
                                            Storage_ParaClassType_List class, \
                                            Storage_FlashInfo_TypeDef **p_Flash, \
                                            Storage_BaseSecInfo_TypeDef **p_Sec, \
                                            StorageIO_TypeDef **StorageIO_API)
{
    if (shell_obj == NULL)
        return;
    
    switch((uint8_t) medium)
    {
        case Internal_Flash:
            shellPrint(shell_obj, "\t[Internal_Flash Selected]\r\n");
            if (!Storage_Monitor.module_enable_reg.bit.internal || \
                !Storage_Monitor.module_init_reg.bit.internal)
            {
                shellPrint(shell_obj, "\t[Internal_Flash Unavaliable]\r\n");
                shellPrint(shell_obj, "\t[Format cnt   : %d]\r\n", Storage_Monitor.InternalFlash_Format_cnt);
                shellPrint(shell_obj, "\t[Buid Tab cnt : %d]\r\n", Storage_Monitor.InternalFlash_BuildTab_cnt);
                return false;
            }
            *p_Flash = &Storage_Monitor.internal_info;
            *StorageIO_API = &InternalFlash_IO;
            break;

        case External_Flash:
            shellPrint(shell_obj, "\t[External_Flash Selected]\r\n");
            if (!Storage_Monitor.module_enable_reg.bit.external || \
                !Storage_Monitor.module_init_reg.bit.external)
            {
                shellPrint(shell_obj, "\t[External_Flash Unavaliable]\r\n");
                shellPrint(shell_obj, "\t[Format cnt   : %d]\r\n", Storage_Monitor.ExternalFlash_Format_cnt);
                shellPrint(shell_obj, "\t[Buid Tab cnt : %d]\r\n", Storage_Monitor.ExternalFlash_BuildTab_cnt);
                return false;
            }
            *p_Flash = &Storage_Monitor.external_info;
            *StorageIO_API = &ExternalFlash_IO;
            break;

        default:
            shellPrint(shell_obj, "\t[Unknow medium type]\r\n");
            return false;
    }

    *p_Sec = Storage_Get_SecInfo(*p_Flash, class);
    if ((*p_Sec == NULL) || \
        ((*p_Sec)->tab_addr == 0) || \
        ((*p_Sec)->tab_size == 0))
    {
        shellPrint(shell_obj, "\tGet section info error\r\n");
        return false;
    }

    return true;
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
           break;
        
        case External_Flash:
            shellPrint(shell_obj, "\t\t[External_Flash Selected]\r\n");
            p_Flash = &Storage_Monitor.external_info;

            if(memcmp(p_Flash->tag, EXTERNAL_STORAGE_PAGE_TAG, EXTERNAL_PAGE_TAG_SIZE))
            {
                shellPrint(shell_obj, "\t\tExternal_Flash Info Error\r\n");
                return;
            }
            break;

        default:
            shellPrint(shell_obj, "medium para error\r\n");
            return;
    }

    shellPrint(shell_obj, "\t\ttotal   size:\t%d\r\n", p_Flash->total_size);
    shellPrint(shell_obj, "\t\tremain  size:\t%d\r\n", p_Flash->remain_size);
    shellPrint(shell_obj, "\t\tstorage size:\t%d\r\n", p_Flash->data_sec_size);
 
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
    
    if(medium == External_Flash)
    {
        shellPrint(shell_obj, "\t[External_Flash Selected]\r\n");
        if (!Storage_Monitor.module_enable_reg.bit.external || !Storage_Monitor.module_init_reg.bit.external)
        {
            shellPrint(shell_obj, "\t[External_Flash Unavaliable]\r\n");
            shellPrint(shell_obj, "\t[Format cnt   : %d]\r\n", Storage_Monitor.ExternalFlash_Format_cnt);
            shellPrint(shell_obj, "\t[Buid Tab cnt : %d]\r\n", Storage_Monitor.ExternalFlash_BuildTab_cnt);
            shellPrint(shell_obj, "\thalt by enable or init state\r\n");
            return;
        }
    }
    else
    {
        shellPrint(shell_obj, "\t[Internal_Flash Selected]\r\n");
        if (!Storage_Monitor.module_enable_reg.bit.internal || !Storage_Monitor.module_init_reg.bit.internal)
        {
            shellPrint(shell_obj, "\t[Internal_Flash Unavaliable]\r\n");
            shellPrint(shell_obj, "\t[Format cnt   : %d]\r\n", Storage_Monitor.InternalFlash_Format_cnt);
            shellPrint(shell_obj, "\t[Buid Tab cnt : %d]\r\n", Storage_Monitor.InternalFlash_BuildTab_cnt);
            shellPrint(shell_obj, "\thalt by enable or init state\r\n");
            return;
        }
    }

    Storage_ClassType_Print(shell_obj);
    if(class > Para_User)
    {
        shellPrint(shell_obj, "\tstorage class arg error\r\n");
        shellPrint(shell_obj, "\ttest halt\r\n");
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

    /* search item first */
    if (Storage_Search(medium, class, test_name).data_addr != 0)
    {
        shellPrint(shell_obj, "\t%s already exist\r\n", test_name);
        return;
    }

    error_code = Storage_CreateItem(medium, class, test_name, test_data, strlen(test_data));
    if(error_code != Storage_Error_None)
    {
        shellPrint(shell_obj, "\t[Storage Test Failed]\r\n");
        shellPrint(shell_obj, "\t[Error_Code]: %s\r\n", Storage_Error_Print(error_code));
        return;
    }
    else
        shellPrint(shell_obj, "\t[Storage Test Done]\r\n");
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, Storage_Test, Storage_Test, Data Storage Test);

static void Storage_Module_Format(Storage_MediumType_List medium)
{
    Shell *shell_obj = Shell_GetInstence();
    
    if ((shell_obj == NULL) || !Storage_Monitor.init_state)
        return;

    switch((uint8_t) medium)
    {
        case Internal_Flash:
            shellPrint(shell_obj, "\t[Internal_Flash Selected]\r\n");

            if (!Storage_Monitor.module_enable_reg.bit.internal || \
                !Storage_Monitor.module_init_reg.bit.internal)
            {
                shellPrint(shell_obj, "\t[Internal_Flash Unavaliable]\r\n");
                shellPrint(shell_obj, "\t[Format cnt   : %d]\r\n", Storage_Monitor.InternalFlash_Format_cnt);
                shellPrint(shell_obj, "\t[Buid Tab cnt : %d]\r\n", Storage_Monitor.InternalFlash_BuildTab_cnt);
                return;
            }
            break;

        case External_Flash:
            shellPrint(shell_obj, "\t[External_Flash Selected]\r\n");
            if (!Storage_Monitor.module_enable_reg.bit.external || \
                !Storage_Monitor.module_init_reg.bit.external)
            {
                shellPrint(shell_obj, "\t[External_Flash Unavaliable]\r\n");
                shellPrint(shell_obj, "\t[Format cnt   : %d]\r\n", Storage_Monitor.ExternalFlash_Format_cnt);
                shellPrint(shell_obj, "\t[Buid Tab cnt : %d]\r\n", Storage_Monitor.ExternalFlash_BuildTab_cnt);
                return;
            }
            break;

        default:
            return;
    }

    shellPrint(shell_obj, "\t[Flash formatting ...]\r\n");
    if (Storage_Format(medium))
    {
        shellPrint(shell_obj, "\t[Flash formatting done]\r\n");
        shellPrint(shell_obj, "\t[Rebuilding storage tab and section]\r\n");

        /* rebuild tab */
        if (!Storage_Build_StorageInfo(External_Flash))
        {
            shellPrint(shell_obj, "\t[Rebuild storage tab and section failed]\r\n");
            return;
        }

        shellPrint(shell_obj, "\t[Rebuild storage tab and section successed]\r\n");
    }
    else
        shellPrint(shell_obj, "\t[Flash Formatting Error]\r\n");
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, Storage_Module_Format, Storage_Module_Format, Storage Format);

static void Storage_Show_ModuleInfo(void)
{
    Storage_ExtFLashDevObj_TypeDef *p_ext_flash = NULL; 
    Shell *shell_obj = Shell_GetInstence();
    void *dev_api = NULL;
    void *dev_obj = NULL;

    if (shell_obj == NULL)
        return;

    if (Storage_Monitor.ExtDev_ptr == NULL)
    {
        shellPrint(shell_obj, "\t[ExtDev pointer NULL]\r\n");
        return;
    }

    shellPrint(shell_obj, "\t[ExtDev pointer %08x]\r\n", Storage_Monitor.ExtDev_ptr);
    p_ext_flash = Storage_Monitor.ExtDev_ptr;

    switch (p_ext_flash->bus_type)
    {
        case Storage_ChipBus_Spi:
            shellPrint(shell_obj, "\t[external flash bus type SPI]\r\n");
            break;
        
        case Storage_ChipBus_None:
        default:
            shellPrint(shell_obj, "\t[none external flash bus matched %d]\r\n", p_ext_flash->bus_type);
            break;
    }

    dev_api = p_ext_flash->dev_api;
    dev_obj = p_ext_flash->dev_obj;
    
    switch (p_ext_flash->chip_type)
    {
        case Storage_ChipType_W25Qxx:
            shellPrint(shell_obj, "\t[external flash chip type W25Qxx]\r\n");

            /* show error code */
            shellPrint(shell_obj, "\t[external flash error code %s]\r\n", Storage_Error_Print(Storage_Monitor.ExternalFlash_Error_Code));
            shellPrint(shell_obj, "\t[external flash format count: %d]\r\n", (Format_Retry_Cnt - Storage_Monitor.ExternalFlash_Format_cnt));
            shellPrint(shell_obj, "\t[external flash re_init count: %d]\r\n", (ExternalModule_ReInit_Cnt - Storage_Monitor.ExternalFlash_ReInit_cnt));
            shellPrint(shell_obj, "\t[external module type %d]\r\n", Storage_Monitor.module_prod_type);
            shellPrint(shell_obj, "\t[external module code %04x]\r\n", Storage_Monitor.module_prod_code);
            break;

        case Storage_Chip_None:
        default:
            shellPrint(shell_obj, "\t[none external flash chip type matched %d]\r\n", p_ext_flash->chip_type);
            break;
    }

    if (dev_api == NULL)
    {
        shellPrint(shell_obj, "\t[dev_api pointer NULL]\r\n");
    }
    else
        shellPrint(shell_obj, "\t[dev_api pointer %08x]\r\n", dev_api);

    if (dev_obj == NULL)
    {
        shellPrint(shell_obj, "\t[dev_obj poniter NULL]\r\n");
    }
    else
    {
        shellPrint(shell_obj, "\t[dev_obj pointer %08x]\r\n", dev_obj);
        shellPrint(shell_obj, "\t[bus_rx     addr %08x]\r\n", To_DevW25Qxx_OBJ(dev_obj)->bus_rx);
        shellPrint(shell_obj, "\t[bus_tx     addr %08x]\r\n", To_DevW25Qxx_OBJ(dev_obj)->bus_tx);
        shellPrint(shell_obj, "\t[bus_trans  addr %08x]\r\n", To_DevW25Qxx_OBJ(dev_obj)->bus_trans);
        shellPrint(shell_obj, "\t[cs_ctl     addr %08x]\r\n", To_DevW25Qxx_OBJ(dev_obj)->cs_ctl);
        shellPrint(shell_obj, "\t[sys_tick   addr %08x]\r\n", To_DevW25Qxx_OBJ(dev_obj)->systick);
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, Storage_Show_ModuleInfo, Storage_Show_ModuleInfo, Storage Format);

static void Storage_Show_FreeSlot(Storage_MediumType_List medium, Storage_ParaClassType_List class)
{
    Storage_FlashInfo_TypeDef *p_Flash = NULL;
    Storage_BaseSecInfo_TypeDef *p_Sec = NULL;
    Storage_FreeSlot_TypeDef *p_FreeSlot = NULL;
    StorageIO_TypeDef *StorageIO_API = NULL;
    uint32_t FreeSlot_addr = 0;
    Shell *shell_obj = Shell_GetInstence();

    if (shell_obj == NULL)
        return;

    shellPrint(shell_obj, "[Storage Shell]Show Freeslot\r\n");
    if(!Storage_Monitor.init_state)
    {
        shellPrint(shell_obj, "\thalt by init state\r\n");
    }

    if (!Storage_Get_Flash_Section_IOAPI(shell_obj, medium, class, &p_Flash, &p_Sec, &StorageIO_API))
    {
        shellPrint(shell_obj, "\t[Flash Section IO_API Get Error]\r\n");
        return;
    }

    FreeSlot_addr = p_Sec->free_slot_addr;
    while(true)
    {
        shellPrint(shell_obj, "\t[FreeSlot Address : %d]\r\n", FreeSlot_addr);
        if (FreeSlot_addr == 0)
        {
            shellPrint(shell_obj, "\t[FreeSlot End]\r\n");
            return;
        }

        /* get free slot info */
        if (!StorageIO_API->read(FreeSlot_addr, page_data_tmp, sizeof(Storage_FreeSlot_TypeDef)))
        {
            shellPrint(shell_obj, "\t[FreeSlot data read error]\r\n");
            return;
        }

        p_FreeSlot = page_data_tmp;
        
        if ((p_FreeSlot->head_tag == STORAGE_SLOT_HEAD_TAG) && \
            (p_FreeSlot->end_tag == STORAGE_SLOT_END_TAG))
        {
            shellPrint(shell_obj, "\t[FreeSlot address      : %d]\r\n", FreeSlot_addr);
            shellPrint(shell_obj, "\t[Next FreeSlot address : %d]\r\n", p_FreeSlot->nxt_addr);
            shellPrint(shell_obj, "\r\n");

            shellPrint(shell_obj, "\t[FreeSlot current size : %d]\r\n", p_FreeSlot->cur_slot_size);
            shellPrint(shell_obj, "\r\n");

            FreeSlot_addr = p_FreeSlot->nxt_addr;
        }
        else
        {
            shellPrint(shell_obj, "\t[FreeSlot frame error]\r\n");
            return;
        }
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, Storage_Show_FreeSlot, Storage_Show_FreeSlot, Storage Show Free Slot);

static void Storage_SearchData(Storage_MediumType_List medium, Storage_ParaClassType_List class, uint8_t *name)
{
    Storage_FlashInfo_TypeDef *p_Flash = NULL;
    Storage_BaseSecInfo_TypeDef *p_Sec = NULL;
    StorageIO_TypeDef *StorageIO_API = NULL;
    Storage_DataSlot_TypeDef DataSlot;
    Storage_Item_TypeDef item;
    uint8_t *crc_buf = NULL;
    uint16_t crc = 0;
    uint16_t crc_len = 0;
    uint16_t data_len = 0;
    uint32_t data_addr = 0;
    Shell *shell_obj = Shell_GetInstence();
    uint8_t *p_read_out = NULL;
    uint16_t i = 0;

    memset(&item, 0, sizeof(item));
    memset(&DataSlot, 0, sizeof(DataSlot));

    if ((shell_obj == NULL) || \
        (name == NULL) || \
        (strlen(name) == 0))
        return;
    
    if (!Storage_Get_Flash_Section_IOAPI(shell_obj, medium, class, &p_Flash, &p_Sec, &StorageIO_API))
    {
        shellPrint(shell_obj, "\t[Flash Section IO_API Get Error]\r\n");
        return;
    }

    item = Storage_Search(medium, class, name);
    if (item.data_addr)
    {
        shellPrint(shell_obj, "\t[tab item %s matched]\r\n", name);
        shellPrint(shell_obj, "\t[data size : %d]\r\n", item.len);
        shellPrint(shell_obj, "\t[data name : %s]\r\n", item.name);
        shellPrint(shell_obj, "\t\r\n");

        data_len = item.len;
        data_addr = item.data_addr;

        while(data_len)
        {
            if (!StorageIO_API->read(data_addr, page_data_tmp, data_len + sizeof(Storage_DataSlot_TypeDef)))
            {
                shellPrint(shell_obj, "\t[Read %s Data failed]\r\n", name);
                return;
            }

            p_read_out = page_data_tmp;
            memcpy(&DataSlot.head_tag, p_read_out, sizeof(DataSlot.head_tag));
            p_read_out += sizeof(DataSlot.head_tag);
            if (DataSlot.head_tag != STORAGE_SLOT_HEAD_TAG)
            {
                shellPrint(shell_obj, "\t[Data %s header tag error]\r\n", name);
                return;
            }

            memcpy(DataSlot.name, p_read_out, STORAGE_NAME_LEN);
            p_read_out += STORAGE_NAME_LEN;
            shellPrint(shell_obj, "\t[Slot name : %s]\r\n", DataSlot.name);

            memcpy(&DataSlot.total_data_size, p_read_out, sizeof(DataSlot.total_data_size));
            p_read_out += sizeof(DataSlot.total_data_size);
            shellPrint(shell_obj, "\t[total data size: %d]\r\n", DataSlot.total_data_size);
            if (DataSlot.total_data_size == 0)
            {
                shellPrint(shell_obj, "\t[Total data size error]\r\n");
                return;
            }

            memcpy(&DataSlot.cur_slot_size, p_read_out, sizeof(DataSlot.cur_slot_size));
            p_read_out += sizeof(DataSlot.cur_slot_size);
            shellPrint(shell_obj, "\t[current slot size: %d]\r\n", DataSlot.cur_slot_size);
            if (DataSlot.cur_slot_size == 0)
            {
                shellPrint(shell_obj, "\t[Current slot data size error]\r\n");
                return;
            }

            memcpy(&DataSlot.nxt_addr, p_read_out, sizeof(DataSlot.nxt_addr));
            p_read_out += sizeof(DataSlot.nxt_addr);
            shellPrint(shell_obj, "\t[next segment data slot address: %d]\r\n", DataSlot.nxt_addr);

            memcpy(&DataSlot.align_size, p_read_out, sizeof(DataSlot.align_size));
            p_read_out += sizeof(DataSlot.align_size);
            shellPrint(shell_obj, "\t[align size : %d]\r\n", DataSlot.align_size);
            shellPrint(shell_obj, "\t[current slot data size %d]\r\n", DataSlot.cur_slot_size - DataSlot.align_size);

            crc_buf = p_read_out;
            crc_len = DataSlot.cur_slot_size;
            crc = Common_CRC16(crc_buf, crc_len);

            p_read_out += DataSlot.cur_slot_size;
            memcpy(&DataSlot.slot_crc, p_read_out, sizeof(DataSlot.slot_crc));
            p_read_out += sizeof(DataSlot.slot_crc);
            if (crc != DataSlot.slot_crc)
            {
                shellPrint(shell_obj, "\t[comput crc %04x slot crc %04x]\r\n", crc, DataSlot.slot_crc);
                shellPrint(shell_obj, "\t[Slot crc error]\r\n");
                return;
            }

            memcpy(&DataSlot.end_tag, p_read_out, sizeof(DataSlot.end_tag));
            if (DataSlot.end_tag != STORAGE_SLOT_END_TAG)
            {
                shellPrint(shell_obj, "\t[Data %s ender tag error]\r\n", name);
                return;
            }

            /* display data as hex */
            shellPrint(shell_obj, "\t[");
            for (i = 0; i < DataSlot.cur_slot_size - DataSlot.align_size; i++)
            {
                shellPrint(shell_obj, " %02x ", crc_buf[i]);
            }
            shellPrint(shell_obj, "]\r\n");

            shellPrint(shell_obj, "\t[ ");
            for (i = 0; i < DataSlot.cur_slot_size - DataSlot.align_size; i++)
            {
                shellPrint(shell_obj, "%c", crc_buf[i]);
            }
            shellPrint(shell_obj, " ]\r\n");

            data_len -= DataSlot.cur_slot_size;
            if (data_len == 0)
            {
                if (DataSlot.nxt_addr == 0)
                {
                    shellPrint(shell_obj, "\t[ ----- END ----- ]\r\n");
                }
                else
                    shellPrint(shell_obj, "\t[ All data dumped but still linked to another slot ]\r\n");

                return;
            }

            if (DataSlot.nxt_addr)
                data_addr = DataSlot.nxt_addr;
        }
    }
    else
        shellPrint(shell_obj, "\t[Storage no %s found in type %d class %d]\r\n", name, medium, class);
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, Storage_Search, Storage_SearchData, Storage Search Data);

static void Storage_Show_Tab(Storage_MediumType_List medium, Storage_ParaClassType_List class)
{
    Storage_FlashInfo_TypeDef *p_Flash = NULL;
    Storage_BaseSecInfo_TypeDef *p_Sec = NULL;
    StorageIO_TypeDef *StorageIO_API = NULL;
    Storage_Item_TypeDef *item_list = NULL;
    uint16_t singal_tab_size = 0;
    uint8_t item_per_tab = 0;
    uint32_t tab_addr = 0;
    uint32_t matched_item_num = 0;
    uint32_t free_item_num = 0;
    uint32_t error_item_num = 0;
    uint8_t *crc_buf = NULL;
    uint16_t crc = 0;
    uint16_t crc_len = 0;
    uint16_t crc_error = 0;
    Storage_FreeSlot_TypeDef *p_FreeSlot = NULL;
    Shell *shell_obj = Shell_GetInstence();
    
    if(shell_obj == NULL)
        return;
    
    shellPrint(shell_obj, "[Storage Shell] Dump Storage Tab\r\n");
    if(!Storage_Monitor.init_state)
    {
        shellPrint(shell_obj, "\thalt by init state\r\n");
    }
    
    if (!Storage_Get_Flash_Section_IOAPI(shell_obj, medium, class, &p_Flash, &p_Sec, &StorageIO_API))
    {
        shellPrint(shell_obj, "\t[Flash Section IO_API Get Error]\r\n");
        return;
    }

    singal_tab_size = p_Sec->tab_size / p_Sec->page_num;
    item_per_tab = singal_tab_size / sizeof(Storage_Item_TypeDef);
    tab_addr = p_Sec->tab_addr;

    for (uint8_t i = 0; i < p_Sec->page_num; i++)
    {
        shellPrint(shell_obj, "\t\t[tab page index : %d]\r\n", i);
        shellPrint(shell_obj, "\t\t[tab page addr  : %d]\r\n", tab_addr);

        if (!StorageIO_API->read(tab_addr, page_data_tmp, singal_tab_size))
        {
            shellPrint(shell_obj, "[...tab address read error...]\r\n");
            shellPrint(shell_obj, "[............ halt ..........]\r\n");
            return;
        }
        else
        {
            /* dump raw tab */
            for (uint16_t t = 0; t < (singal_tab_size / 16); t++)
            {
                shellPrint(shell_obj, "[");
                for (uint8_t t_i = 0; t_i < 4; t_i++)
                    shellPrint(shell_obj, " %08x ", *((uint32_t *)&page_data_tmp[(t_i * 4) + (t * 16)]));
                shellPrint(shell_obj, "]\r\n");
            }

            /* convert raw data to tab item list */
            item_list = page_data_tmp;
            crc_error = 0;
            for (uint8_t j = 0; j < item_per_tab; j++)
            {
                if ((item_list[j].head_tag == STORAGE_ITEM_HEAD_TAG) && \
                    (item_list[j].end_tag == STORAGE_ITEM_END_TAG))
                {
                    crc_buf = ((uint8_t *)&item_list[j]) + sizeof(item_list[j].head_tag);
                    crc_len = sizeof(Storage_Item_TypeDef);
                    crc_len -= sizeof(item_list[j].head_tag);
                    crc_len -= sizeof(item_list[j].end_tag);
                    crc_len -= sizeof(item_list[j].crc16);
                    crc = Common_CRC16(crc_buf, crc_len);

                    shellPrint(shell_obj, "\t\t[item class    : %d]\r\n", item_list[j].class);
                    shellPrint(shell_obj, "\t\t[item name     : %s]\r\n", item_list[j].name);
                    shellPrint(shell_obj, "\t\t[item address  : %d]\r\n", item_list[j].data_addr);
                    shellPrint(shell_obj, "\t\t[item data len : %d]\r\n", item_list[j].len);
                    shellPrint(shell_obj, "\r\n");

                    if (crc == item_list[j].crc16)
                    {
                        if (memcmp(item_list[j].name, STORAGE_FREEITEM_NAME, strlen(STORAGE_FREEITEM_NAME)) == 0)
                        {
                            free_item_num ++;
                        }
                        else
                            matched_item_num ++;
                    }
                    else
                    {
                        crc_error ++;
                        error_item_num ++;
                    }
                }
            }

            if (crc_error)
                shellPrint(shell_obj, "\t\t\t[error item num in tab : %d]\r\n", crc_error);
        }

        tab_addr += singal_tab_size;
    }
 
    shellPrint(shell_obj, "\t[all tab size          : %d]\r\n", p_Sec->tab_size);
    shellPrint(shell_obj, "\t[singal tab size       : %d]\r\n", singal_tab_size);
    shellPrint(shell_obj, "\t[item per tab          : %d]\r\n", item_per_tab);
    shellPrint(shell_obj, "\t[tab start addr        : %d]\r\n", tab_addr);
    shellPrint(shell_obj, "\t[tab address           : %d]\r\n", p_Sec->tab_addr);
    shellPrint(shell_obj, "\t[tab page num          : %d]\r\n", p_Sec->page_num);
    shellPrint(shell_obj, "\t[para num              : %d]\r\n", p_Sec->para_num);
    shellPrint(shell_obj, "\t[para size             : %d]\r\n", p_Sec->para_size);
    shellPrint(shell_obj, "\r\n");

    shellPrint(shell_obj, "\t[matched data item num : %d]\r\n", matched_item_num);
    shellPrint(shell_obj, "\t[matched free item num : %d]\r\n", free_item_num);
    shellPrint(shell_obj, "\t[error item num        : %d]\r\n", error_item_num);
    
    /* show free address */
    if (!StorageIO_API->read(p_Sec->free_slot_addr, page_data_tmp, sizeof( Storage_FreeSlot_TypeDef)))
    {
        shellPrint(shell_obj, "[Free slot info read failed]\r\n");
        shellPrint(shell_obj, "[.......... halt ..........]\r\n");
        return;
    }

    p_FreeSlot = (Storage_FreeSlot_TypeDef *)page_data_tmp;
    if ((p_FreeSlot->head_tag == STORAGE_SLOT_HEAD_TAG) && \
        (p_FreeSlot->end_tag == STORAGE_SLOT_END_TAG))
    {
        shellPrint(shell_obj, "\t[free slot address     : %d]\r\n", p_Sec->free_slot_addr);
        shellPrint(shell_obj, "\t[current free slot size: %d]\r\n", p_FreeSlot->cur_slot_size);
        shellPrint(shell_obj, "\t[next free slot address: %d]\r\n", p_FreeSlot->nxt_addr);
    }
    else
        shellPrint(shell_obj, "\t[free slot read error]\r\n");

    if (matched_item_num != p_Sec->para_num)
        shellPrint(shell_obj, "[warnning stored parameter number]\r\n");

    shellPrint(shell_obj, "\r\n");
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, Storage_DumpTab, Storage_Show_Tab, Storage dump Tab);

static void Storage_Dump_DataSection(Storage_MediumType_List medium, Storage_ParaClassType_List class)
{
    uint32_t flash_sector_size = 0;
    uint32_t remain_dump_size = 0;
    uint32_t dump_size = 0;
    uint32_t dump_addr = 0;
    StorageIO_TypeDef *StorageIO_API = NULL;
    Storage_FlashInfo_TypeDef *p_Flash = NULL;
    Storage_BaseSecInfo_TypeDef *p_Sec = NULL;
    Shell *shell_obj = Shell_GetInstence();
    Storage_ExtFLashDevObj_TypeDef *ext_dev = NULL;

    if (shell_obj == NULL)
        return;
    
    shellPrint(shell_obj, "[Storage Shell] Dump Data Section\r\n");
    if(!Storage_Monitor.init_state)
    {
        shellPrint(shell_obj, "\thalt by init state\r\n");
        return;
    }

    if (!Storage_Get_Flash_Section_IOAPI(shell_obj, medium, class, &p_Flash, &p_Sec, &StorageIO_API))
    {
        shellPrint(shell_obj, "\t[Flash Section IO_API Get Error]\r\n");
        return;
    }

    switch (medium)
    {
        case Internal_Flash:
        // flash_sector_size = 
            shellPrint(shell_obj, "\t[Still In Developping]\r\n");
            return;

        case External_Flash:
            if (Storage_Monitor.ExtDev_ptr)
            {
                ext_dev = (Storage_ExtFLashDevObj_TypeDef *)Storage_Monitor.ExtDev_ptr;
                if (ext_dev->chip_type == Storage_ChipType_W25Qxx)
                {
                    flash_sector_size = To_DevW25Qxx_API(ext_dev->dev_api)->info(ext_dev->dev_obj).subsector_size;
                    break;
                }
            }
            return;

        default:
            return;
    }

    remain_dump_size = p_Sec->data_sec_size;
    dump_size = remain_dump_size;
    dump_addr = p_Sec->data_sec_addr;
    while(remain_dump_size)
    {
        if (dump_size >= flash_sector_size)
            dump_size = flash_sector_size;

        if (!StorageIO_API->read(dump_addr, page_data_tmp, dump_size))
        {
            shellPrint(shell_obj, "\tData Section Address : %d\r\n", dump_addr);
            shellPrint(shell_obj, "\tDump Size            : %d\r\n", dump_size);
            shellPrint(shell_obj, "\tData Section Read Error\r\n");
            shellPrint(shell_obj, "\thalt\r\n");
            return;
        }

        for (uint16_t i = 0; i < (dump_size / 16); i++)
        {
            shellPrint(shell_obj, "[");
            for (uint8_t j = 0; j < 4; j++)
                shellPrint(shell_obj, " %08x ", *((uint32_t *)&page_data_tmp[(i * 16) + (j * 4)]));
            shellPrint(shell_obj, "]\r\n");
        }

        dump_addr += dump_size;
        if (remain_dump_size >= dump_size)
            remain_dump_size -= dump_size;
        dump_size = remain_dump_size;
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, Storage_DumpSection, Storage_Dump_DataSection, Storage dump data section);

static void Storage_UpdateData(Storage_MediumType_List medium, Storage_ParaClassType_List class, char *test_name, char *test_data)
{
    Shell *shell_obj = Shell_GetInstence();
    StorageIO_TypeDef *StorageIO_API = NULL;
    Storage_FlashInfo_TypeDef *p_Flash = NULL;
    Storage_BaseSecInfo_TypeDef *p_Sec = NULL;
    Storage_Item_TypeDef item;
    Storage_ErrorCode_List update_error_code = Storage_Error_None;
    
    memset(&item, 0, sizeof(item));

    if ((shell_obj == NULL) || \
        (test_name == NULL) || \
        (test_data == NULL) || \
        (strlen(test_name) == 0) || \
        (strlen(test_data) == 0))
        return;

    if (!Storage_Get_Flash_Section_IOAPI(shell_obj, medium, class, &p_Flash, &p_Sec, &StorageIO_API))
    {
        shellPrint(shell_obj, "\t[Flash Section IO_API Get Error]\r\n");
        return;
    }

    item = Storage_Search(medium, class, test_name);
    if ((item.head_tag != STORAGE_ITEM_HEAD_TAG) || \
        (item.end_tag != STORAGE_ITEM_END_TAG) || \
        (item.data_addr == 0))
    {
        shellPrint(shell_obj, "\t[Item slot error]\r\n");
        return;
    }

    update_error_code = Storage_SlotData_Update(medium, class, item.data_addr, test_data, strlen(test_data)); 
    if (update_error_code != Storage_Error_None)
    {
        shellPrint(shell_obj, "\t[Data update failed %s]\r\n", Storage_Error_Print(update_error_code));
        return;
    }
    else
        shellPrint(shell_obj, "\t[DataSlot Update accomplished]\r\n");
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, Storage_UpdateData, Storage_UpdateData, Storage update data);

static void Storage_DeleteData_Test(Storage_MediumType_List medium, Storage_ParaClassType_List class, char *test_name)
{
    Shell *shell_obj = Shell_GetInstence();
    StorageIO_TypeDef *StorageIO_API = NULL;
    Storage_FlashInfo_TypeDef *p_Flash = NULL;
    Storage_BaseSecInfo_TypeDef *p_Sec = NULL;
    Storage_Item_TypeDef item;
    
    if (shell_obj == NULL)
        return;
    
    if (!Storage_Get_Flash_Section_IOAPI(shell_obj, medium, class, &p_Flash, &p_Sec, &StorageIO_API))
    {
        shellPrint(shell_obj, "\t[Flash Section IO_API Get Error]\r\n");
        return;
    }

    /* search */
    item = Storage_Search(medium, class, test_name);
    if ((item.head_tag != STORAGE_ITEM_HEAD_TAG) || \
        (item.end_tag != STORAGE_ITEM_END_TAG) || \
        (item.data_addr == 0))
    {
        shellPrint(shell_obj, "\t[Item slot error]\r\n");
        return;
    }

    if (!Storage_DeleteAllDataSlot(item.data_addr, item.name, item.len, p_Sec, StorageIO_API))
    {
        shellPrint(shell_obj, "\t[Item Delete Error]\r\n");
    }
    else
        shellPrint(shell_obj, "\t[Item Delete Accomplished]\r\n");
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, Storage_DeleteData, Storage_DeleteData_Test, Storage delete data);
