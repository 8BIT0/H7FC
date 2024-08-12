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
#include "HW_Def.h"
#include "Srv_OsCommon.h"
#include "debug_util.h"

#define STORAGE_DEBUG 1

#define InternalFlash_BootDataSec_Size (4 Kb)
#define InternalFlash_SysDataSec_Size (16 Kb)
#define InternalFlash_UserDataSec_Size (32 Kb)

#define Storage_TabSize Flash_Storage_TabSize
#define Storage_InfoPageSize Flash_Storage_InfoPageSize

#define Item_Capacity_Per_Tab (Storage_TabSize / sizeof(Storage_Item_TypeDef))
#if defined STM32H743xx
static SPI_HandleTypeDef ExtFlash_Bus_InstObj;
#elif defined AT32F435RGT7
void *ExtFlash_Bus_InstObj = NULL;
#endif

#define STORAGE_TAG "[ STORAGE INFO ] "
#define STORAGE_INFO(fmt, ...) Debug_Print(&DebugPort, STORAGE_TAG , fmt, ##__VA_ARGS__)

/* internal vriable */
Storage_Monitor_TypeDef Storage_Monitor;
static uint8_t page_data_tmp[Storage_TabSize * 2] __attribute__((aligned(4))) = {0};
static uint8_t flash_write_tmp[Storage_TabSize * 2] __attribute__((aligned(4))) __attribute__((section(".Perph_Section"))) = {0};
static uint8_t flash_read_tmp[Storage_TabSize * 2] __attribute__((aligned(4))) __attribute__((section(".Perph_Section"))) = {0};

static bool Storage_Clear_Tab(uint32_t addr, uint32_t tab_num);
static bool Storage_Establish_Tab(Storage_ParaClassType_List class);

static bool Storage_ExtFlash_ParaSec_Read(uint32_t addr_offset, uint8_t *p_data, uint32_t len);
static bool Storage_ExtFlash_ParaSec_Write(uint32_t addr_offset, uint8_t *p_data, uint32_t len);
static bool Storage_ExtFlash_ParaSec_Erase(uint32_t addr_offset, uint32_t len);
static bool Storage_ExtFlash_EraseAll(void);

/* internal function */
static bool Storage_External_Chip_W25Qxx_SelectPin_Ctl(bool state);
static uint16_t Storage_External_Chip_W25Qxx_BusTx(uint8_t *p_data, uint16_t len, uint32_t time_out);
static uint16_t Storage_External_Chip_W25Qxx_BusRx(uint8_t *p_data, uint16_t len, uint32_t time_out);
static uint16_t Storage_External_Chip_W25Qxx_BusTrans(uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t time_out);
static bool Storage_Build_StorageInfo(void);
static bool Storage_Get_StorageInfo(void);
static bool Storage_Format(void);
static bool Storage_Compare_ItemSlot_CRC(const Storage_Item_TypeDef item);
static bool Storage_Comput_ItemSlot_CRC(Storage_Item_TypeDef *p_item);
static Storage_BaseSecInfo_TypeDef* Storage_Get_SecInfo(Storage_FlashInfo_TypeDef *info, Storage_ParaClassType_List class);
static bool Storage_DeleteSingleDataSlot(uint32_t slot_addr, uint8_t *p_data, Storage_BaseSecInfo_TypeDef *p_Sec);
static Storage_ErrorCode_List Storage_FreeSlot_CheckMerge(uint32_t slot_addr, Storage_FreeSlot_TypeDef *slot_info, Storage_BaseSecInfo_TypeDef *p_Sec);
static bool Storage_Link_FreeSlot(uint32_t front_free_addr, uint32_t behind_free_addr, uint32_t new_free_addr, Storage_FreeSlot_TypeDef *new_free_slot);
static Storage_ErrorCode_List Storage_ItemSlot_Update(uint32_t tab_addr, uint8_t item_index, Storage_BaseSecInfo_TypeDef *p_Sec, Storage_Item_TypeDef item);

/* external function */
static bool Storage_Init(Storage_ExtFLashDevObj_TypeDef *ExtDev);
static Storage_ItemSearchOut_TypeDef Storage_Search(Storage_ParaClassType_List _class, const char *name);
static Storage_ErrorCode_List Storage_DeleteItem(Storage_ParaClassType_List _class, const char *name, uint32_t size);
static Storage_ErrorCode_List Storage_CreateItem(Storage_ParaClassType_List _class, const char *name, uint8_t *p_data, uint16_t size);
static Storage_ErrorCode_List Storage_SlotData_Update(Storage_ParaClassType_List _class, storage_handle data_slot_hdl, uint8_t *p_data, uint16_t size);
static Storage_ErrorCode_List Storage_Get_Data(Storage_ParaClassType_List _class, Storage_Item_TypeDef item, uint8_t *p_data, uint16_t size);
static Storage_ErrorCode_List Storage_Get_DevInfo(Storage_ExtFLashDevObj_TypeDef *info);
static bool Storage_Write_Section(uint32_t addr, uint8_t *p_data, uint16_t len);
static bool Storage_Read_Section(uint32_t addr, uint8_t *p_data, uint16_t len);
static bool Storage_Erase_Section(uint32_t addr, uint16_t len);

static bool Storage_Firmware_Format(Storage_FirmwareType_List type);
static bool Storage_Frimware_Read(Storage_FirmwareType_List type, uint32_t addr_offset, uint8_t *p_data, uint16_t size);
static bool Storage_Firmware_Write(Storage_MediumType_List medium, Storage_FirmwareType_List type, uint32_t addr_offset, uint8_t *p_data, uint16_t size);

Storage_TypeDef Storage = {
    .init = Storage_Init,
    .search = Storage_Search,
    .create = Storage_CreateItem,
    .get = Storage_Get_Data,
    .update = Storage_SlotData_Update,
    .get_dev_info = Storage_Get_DevInfo,

    .write_section = Storage_Write_Section,
    .read_section = Storage_Read_Section,
    .erase_section = Storage_Erase_Section,

    .format_firmware = Storage_Firmware_Format,
    .read_firmware = Storage_Frimware_Read,
    .write_firmware = Storage_Firmware_Write,
};

static bool Storage_Init(Storage_ExtFLashDevObj_TypeDef *ExtDev)
{
    void *ext_flash_bus_cfg = NULL;
    uint8_t extmodule_init_state;
    memset(&Storage_Monitor, 0, sizeof(Storage_Monitor));

    Storage_Monitor.init_state = false;
    if (!BspFlash.init())
        return false;

    /* external flash init */
#if (FLASH_CHIP_STATE == ON)
    if (ExtDev && (ExtDev->chip_type != Storage_Chip_None))
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
                if (ExtFlash_Bus_Api.init(To_NormalSPI_Obj(ext_flash_bus_cfg), &ExtFlash_Bus_InstObj) && \
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
                                ExtDev->start_addr  = W25QXX_BASE_ADDRESS;
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
                                if (!Storage_Get_StorageInfo())
                                {
reformat_external_flash_info:
                                    if (Storage_Monitor.ExternalFlash_Format_cnt)
                                    {
                                        /* format storage device */
                                        if (!Storage_Format())
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
                                            if (Storage_Build_StorageInfo())
                                            {
                                                Storage_Monitor.ExternalFlash_BuildTab_cnt ++;
                                                Storage_Monitor.init_state = true;

                                                /* after tab builded read storage info again */
                                                goto reupdate_external_flash_info;
                                            }
                                        }
                                    }
                                }
                                else
                                    Storage_Monitor.init_state = true;
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
#endif

    return Storage_Monitor.init_state;
}

static Storage_ErrorCode_List Storage_Get_DevInfo(Storage_ExtFLashDevObj_TypeDef *info)
{
    Storage_ExtFLashDevObj_TypeDef *p_dev = NULL;

    if (info)
    {
        memset(info, 0, sizeof(Storage_ExtFLashDevObj_TypeDef));
        if (Storage_Monitor.init_state && Storage_Monitor.ExtDev_ptr)
        {
            p_dev = (Storage_ExtFLashDevObj_TypeDef *)Storage_Monitor.ExtDev_ptr;
            memcpy(info, p_dev, sizeof(Storage_ExtFLashDevObj_TypeDef));
            return Storage_Error_None;
        }

        return Storage_ModuleInit_Error;
    }

    return Storage_ExtDevObj_Error;
}

static bool Storage_Format(void)
{
    uint32_t size = 0;
    uint8_t default_data = 0;
    uint32_t read_time = 0;
    uint32_t remain_size = 0;
    uint32_t addr_offset = From_Start_Address;
        
    size = Storage_TabSize;
    default_data = ExtFlash_Storage_DefaultData;
    remain_size = ExtFlash_Storage_TotalSize;
    read_time = ExtFlash_Storage_TotalSize / Storage_TabSize;
    if (ExtFlash_Storage_TotalSize % Storage_TabSize)
        read_time ++;

    for(uint32_t i = 0; i < read_time; i++)
    {
        if((remain_size != 0) && (remain_size < size))
            size = remain_size;

        if (!Storage_ExtFlash_ParaSec_Erase(addr_offset, size))
            return false;

        if (!Storage_ExtFlash_ParaSec_Read(addr_offset, page_data_tmp, size))
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

    return false;
}

static bool Storage_Check_Tab(Storage_BaseSecInfo_TypeDef *sec_info)
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

    if (sec_info)
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
                !Storage_ExtFlash_ParaSec_Read(free_slot_addr, page_data_tmp, Storage_TabSize))
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
                if (!Storage_ExtFlash_ParaSec_Read(tab_addr, page_data_tmp, sec_info->tab_size / sec_info->page_num))
                    return false;
            
                p_ItemList = (Storage_Item_TypeDef *)page_data_tmp;
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

static bool Storage_Get_StorageInfo(void)
{
    Storage_FlashInfo_TypeDef *p_Info = NULL;
    Storage_FlashInfo_TypeDef Info_r;
    uint16_t crc = 0;
    uint16_t crc_read = 0;
    char flash_tag[INTERNAL_PAGE_TAG_SIZE + EXTERNAL_PAGE_TAG_SIZE];

    memset(flash_tag, '\0', sizeof(flash_tag));
    memset(&Info_r, 0, sizeof(Storage_FlashInfo_TypeDef));

    memcpy(flash_tag, EXTERNAL_STORAGE_PAGE_TAG, EXTERNAL_PAGE_TAG_SIZE);
    p_Info = &Storage_Monitor.external_info;
    
    if (Storage_ExtFlash_ParaSec_Read(From_Start_Address, page_data_tmp, Storage_TabSize))
    {
        /* check internal storage tag */
        Info_r = *(Storage_FlashInfo_TypeDef *)page_data_tmp;
        
        /* check storage tag */
        /* check boot / sys / user  start addr */
        if ((strcmp((const char *)Info_r.tag, flash_tag) != 0) || \
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
        if (Storage_Check_Tab(&Info_r.boot_sec) && \
            Storage_Check_Tab(&Info_r.sys_sec) && \
            Storage_Check_Tab(&Info_r.user_sec))
        {
            memcpy(p_Info, &Info_r, sizeof(Storage_FlashInfo_TypeDef));
            return true;
        }
    }

    return false;
}

static bool Storage_Clear_Tab(uint32_t addr, uint32_t tab_num)
{
    uint32_t addr_tmp = 0;

    if ((addr == 0) || \
        (tab_num == 0))
        return false;

    memset(page_data_tmp, 0, Storage_TabSize);
    addr_tmp = addr;
    
    for(uint32_t i = 0; i < tab_num; i++)
    {
        if (!Storage_ExtFlash_ParaSec_Write(addr_tmp, page_data_tmp, Storage_TabSize))
            return false;

        addr_tmp += Storage_TabSize;
    }

    return true;
}

/* 
 * if matched return data slot address 
 * else return 0
 */
static Storage_ItemSearchOut_TypeDef Storage_Search(Storage_ParaClassType_List _class, const char *name)
{
    Storage_BaseSecInfo_TypeDef *p_Sec = NULL;
    Storage_Item_TypeDef *item_list = NULL;
    Storage_Item_TypeDef *p_item = NULL;
    uint32_t tab_addr = 0;
    Storage_ItemSearchOut_TypeDef ItemSearch;

    memset(&ItemSearch, 0, sizeof(ItemSearch));

    if (!Storage_Monitor.init_state || \
        (name == NULL) || \
        (strlen(name) == 0) || \
        (_class > Para_User))
        return ItemSearch;

    p_Sec = Storage_Get_SecInfo(&Storage_Monitor.external_info, _class);

    if ((p_Sec == NULL) || \
        (p_Sec->para_num == 0) || \
        (p_Sec->para_size == 0))
        return ItemSearch;

    tab_addr = p_Sec->tab_addr;
    /* tab traverse */
    for (uint8_t tab_i = 0; tab_i < p_Sec->page_num; tab_i ++)
    {
        if (!Storage_ExtFlash_ParaSec_Read(tab_addr, page_data_tmp, (p_Sec->tab_size / p_Sec->page_num)))
            return ItemSearch;
    
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
                    ItemSearch.item_addr = tab_addr;
                    ItemSearch.item_index = item_i;
                    ItemSearch.item = *p_item;
                    return ItemSearch;
                }
            }
        }
    
        /* update tab address */
        tab_addr += (p_Sec->tab_size / p_Sec->page_num);
    }

    return ItemSearch;
}

static Storage_ErrorCode_List Storage_ItemSlot_Update(uint32_t tab_addr, uint8_t item_index, Storage_BaseSecInfo_TypeDef *p_Sec, Storage_Item_TypeDef item)
{
    Storage_Item_TypeDef *ItemList = NULL;

    if ((tab_addr == 0) || \
        (p_Sec == NULL) || \
        (item.head_tag != STORAGE_ITEM_HEAD_TAG) || \
        (item.end_tag != STORAGE_ITEM_END_TAG) || \
        (item.data_addr == 0) || \
        (item.data_addr > (p_Sec->data_sec_addr + p_Sec->data_sec_size)) || \
        (item.data_addr < p_Sec->data_sec_addr))
        return Storage_Param_Error;

    if (!Storage_ExtFlash_ParaSec_Read(tab_addr, &page_data_tmp[Storage_TabSize], Storage_TabSize))
        return Storage_Read_Error;

    ItemList = (Storage_Item_TypeDef *)&page_data_tmp[Storage_TabSize];
    ItemList[item_index] = item;

    if (!Storage_ExtFlash_ParaSec_Write(tab_addr, (uint8_t *)ItemList, Storage_TabSize))
        return Storage_Write_Error;

    return Storage_Error_None;
}

static Storage_ErrorCode_List Storage_Get_Data(Storage_ParaClassType_List class, Storage_Item_TypeDef item, uint8_t *p_data, uint16_t size)
{
    uint32_t data_len = 0;
    uint32_t data_addr = 0;
    uint8_t *p_read_out = NULL;
    uint8_t *crc_buf = NULL;
    uint16_t crc_len = 0;
    uint16_t crc = 0;
    Storage_DataSlot_TypeDef DataSlot;
    uint8_t *p_data_start = NULL;

    memset(&DataSlot, 0, sizeof(Storage_DataSlot_TypeDef));
    if (item.data_addr && p_data && size)
    {
        data_len = item.len;
        data_addr = item.data_addr;

        while(data_len)
        {
            if (!Storage_ExtFlash_ParaSec_Read(data_addr, page_data_tmp, data_len + sizeof(Storage_DataSlot_TypeDef)))
                return Storage_GetData_Error;

            p_read_out = page_data_tmp;
            memcpy(&DataSlot.head_tag, p_read_out, sizeof(DataSlot.head_tag));
            p_read_out += sizeof(DataSlot.head_tag);
            if (DataSlot.head_tag != STORAGE_SLOT_HEAD_TAG)
                return Storage_GetData_Error;

            memcpy(DataSlot.name, p_read_out, STORAGE_NAME_LEN);
            p_read_out += STORAGE_NAME_LEN;

            memcpy(&DataSlot.total_data_size, p_read_out, sizeof(DataSlot.total_data_size));
            p_read_out += sizeof(DataSlot.total_data_size);
            if (DataSlot.total_data_size == 0)
                return Storage_GetData_Error;

            memcpy(&DataSlot.cur_slot_size, p_read_out, sizeof(DataSlot.cur_slot_size));
            p_read_out += sizeof(DataSlot.cur_slot_size);
            if (DataSlot.cur_slot_size == 0)
                return Storage_GetData_Error;

            memcpy(&DataSlot.nxt_addr, p_read_out, sizeof(DataSlot.nxt_addr));
            p_read_out += sizeof(DataSlot.nxt_addr);

            memcpy(&DataSlot.align_size, p_read_out, sizeof(DataSlot.align_size));
            p_read_out += sizeof(DataSlot.align_size);
            p_data_start = p_read_out;

            crc_buf = p_read_out;
            crc_len = DataSlot.cur_slot_size;
            crc = Common_CRC16(crc_buf, crc_len);

            p_read_out += DataSlot.cur_slot_size;
            memcpy(&DataSlot.slot_crc, p_read_out, sizeof(DataSlot.slot_crc));
            p_read_out += sizeof(DataSlot.slot_crc);
            if (crc != DataSlot.slot_crc)
                return Storage_GetData_Error;

            memcpy(&DataSlot.end_tag, p_read_out, sizeof(DataSlot.end_tag));
            if (DataSlot.end_tag != STORAGE_SLOT_END_TAG)
                return Storage_GetData_Error;

            memcpy(p_data, p_data_start, DataSlot.cur_slot_size - DataSlot.align_size);
            data_len -= DataSlot.cur_slot_size;

            if (DataSlot.nxt_addr)
            {
                p_data += DataSlot.cur_slot_size - DataSlot.align_size;
                data_addr = DataSlot.nxt_addr;
            }
            else
            {
                if (data_len == 0)
                    break;

                return Storage_GetData_Error;
            }
        }

        return Storage_Error_None;
    }

    return Storage_GetData_Error;
}

static Storage_ErrorCode_List Storage_SlotData_Update(Storage_ParaClassType_List _class, storage_handle data_slot_hdl, uint8_t *p_data, uint16_t size)
{
    Storage_BaseSecInfo_TypeDef *p_Sec = NULL;
    Storage_DataSlot_TypeDef *p_slotdata = NULL;
    uint8_t *p_read_tmp = page_data_tmp;
    uint8_t *crc_buf = NULL;
    uint16_t crc = 0;
    uint32_t read_addr = 0;
    uint32_t read_size = 0;
    uint32_t valid_data_size = 0;
    uint8_t align_byte = 0;

    if (!Storage_Monitor.init_state || \
        (_class > Para_User) || \
        (data_slot_hdl == 0) || \
        (p_data == NULL) || \
        (size == 0))
        return Storage_Param_Error;

    p_Sec = Storage_Get_SecInfo(&Storage_Monitor.external_info, _class);
    
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

    /* get data slot first */
    if (!Storage_ExtFlash_ParaSec_Read(read_addr, p_read_tmp, sizeof(Storage_DataSlot_TypeDef)))
        return Storage_Read_Error;
    
    /* get data size */
    p_slotdata = (Storage_DataSlot_TypeDef *)p_read_tmp;
    if ((p_slotdata->head_tag != STORAGE_SLOT_HEAD_TAG) || \
        (p_slotdata->total_data_size == 0) || \
        (p_slotdata->total_data_size > p_Sec->data_sec_size))
        return Storage_DataInfo_Error;

    if (p_slotdata->total_data_size != (size + align_byte))
        return Storage_Update_DataSize_Error;

    read_size = p_slotdata->total_data_size + sizeof(Storage_DataSlot_TypeDef);
    p_read_tmp = page_data_tmp;
    memset(p_read_tmp, 0, sizeof(Storage_DataSlot_TypeDef));
    p_slotdata = NULL;
    while(true)
    {
        p_read_tmp = page_data_tmp;
        p_slotdata = (Storage_DataSlot_TypeDef *)p_read_tmp;

        /* get data from handle */
        if (!Storage_ExtFlash_ParaSec_Read(read_addr, p_read_tmp, read_size))
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

        crc_buf = p_read_tmp;
        memcpy(p_read_tmp, p_data, (p_slotdata->cur_slot_size - p_slotdata->align_size));
        p_read_tmp += p_slotdata->cur_slot_size - p_slotdata->align_size;

        if (p_slotdata->align_size)
            /* set align byte */
            memset(p_read_tmp, 0, p_slotdata->align_size);
        
        p_read_tmp += p_slotdata->align_size;
        /* update crc */
        crc = Common_CRC16(crc_buf, p_slotdata->cur_slot_size) ;
        
        valid_data_size += p_slotdata->cur_slot_size - p_slotdata->align_size;
        if (p_slotdata->nxt_addr == 0)
        {
            /* all data has been read out from the data slot in destination section */
            /* compare update data size and valid_data_size */
            if (size != valid_data_size)
                return Storage_Update_DataSize_Error;
        }

        memcpy(p_read_tmp, &crc, sizeof(crc));
        
        p_data += p_slotdata->cur_slot_size - p_slotdata->align_size;
        p_read_tmp += sizeof(p_slotdata->slot_crc);

        if (*(uint32_t *)p_read_tmp != STORAGE_SLOT_END_TAG)
            return Storage_DataInfo_Error;

        if ((p_slotdata->nxt_addr == 0) && \
            (size == valid_data_size))
        {
            /* update data to flash */
            if (!Storage_ExtFlash_ParaSec_Write(read_addr, page_data_tmp, p_slotdata->cur_slot_size + sizeof(Storage_DataSlot_TypeDef)))
                return Storage_Write_Error;

            /* update accomplish */
            return Storage_Error_None;
        }

        read_addr = p_slotdata->nxt_addr;
    }

    return Storage_Error_None;
}

static bool Storage_Link_FreeSlot(uint32_t front_free_addr, uint32_t behind_free_addr, uint32_t new_free_addr, Storage_FreeSlot_TypeDef *new_free_slot)
{
    Storage_FreeSlot_TypeDef front_slot;
    Storage_FreeSlot_TypeDef behind_slot;

    if ((new_free_addr == 0) || \
        (front_free_addr == 0) || \
        (behind_free_addr == 0) || \
        (front_free_addr >= new_free_addr) || \
        (behind_free_addr <= new_free_addr) || \
        (new_free_slot == NULL))
        return false;

    memset(&front_slot, 0, sizeof(Storage_FreeSlot_TypeDef));
    memset(&behind_slot, 0, sizeof(Storage_FreeSlot_TypeDef));

/* 
 *
 *       address N                    address X                   address Y
 * _____________________        _____________________        ______________________
 * |  front free slot  |   ———→ |   new free slot   |   ———→ |  behind free slot  |   ———→ ... ...
 * |                   |   |    |                   |   |    |                    |   |
 * |____next_addr_X____|   |    |____next_addr_Y____|   |    |_____next_addr_Z____|   |
 *          |______________|             |______________|              |______________|
 * 
 */

    if (!Storage_ExtFlash_ParaSec_Read(front_free_addr, (uint8_t *)&front_slot, sizeof(Storage_FreeSlot_TypeDef)))
        return false;

    /* link free slot address */
    front_slot.nxt_addr = new_free_addr;
    new_free_slot->nxt_addr = behind_free_addr;

    if (!Storage_ExtFlash_ParaSec_Write(front_free_addr, (uint8_t *)&front_slot, sizeof(Storage_FreeSlot_TypeDef)))
        return false;

    if (!Storage_ExtFlash_ParaSec_Write(new_free_addr, (uint8_t *)new_free_slot, sizeof(Storage_FreeSlot_TypeDef)))
        return false;

    return true;
}

/* developping & untested */
static Storage_ErrorCode_List Storage_FreeSlot_CheckMerge(uint32_t slot_addr, Storage_FreeSlot_TypeDef *slot_info, Storage_BaseSecInfo_TypeDef *p_Sec)
{
    Storage_FreeSlot_TypeDef FreeSlot_Info;
    uint32_t front_freeslot_addr = 0;
    uint32_t behind_freeslot_addr = 0;
    uint32_t ori_freespace_size = 0;

    if ((p_Sec == NULL) || \
        (slot_info == NULL) || \
        (slot_addr < p_Sec->data_sec_addr) || \
        (slot_addr > (p_Sec->data_sec_addr + p_Sec->data_sec_size)))
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
        if (!Storage_ExtFlash_ParaSec_Read(front_freeslot_addr, (uint8_t *)&FreeSlot_Info, sizeof(FreeSlot_Info)))
            return Storage_Read_Error;

        if ((FreeSlot_Info.head_tag != STORAGE_SLOT_HEAD_TAG) || \
            (FreeSlot_Info.end_tag != STORAGE_SLOT_END_TAG))
            return Storage_FreeSlot_Info_Error;

        behind_freeslot_addr = FreeSlot_Info.nxt_addr;
        p_Sec->free_space_size += slot_info->cur_slot_size;

        /* circumstance 1: new free slot in front of the old free slot */
        if (slot_addr + slot_info->cur_slot_size == front_freeslot_addr)
        {
            slot_info->nxt_addr = FreeSlot_Info.nxt_addr;
            slot_info->cur_slot_size += FreeSlot_Info.cur_slot_size + sizeof(Storage_FreeSlot_TypeDef);

            memset(&FreeSlot_Info, 0, sizeof(FreeSlot_Info));

            /* write to front freeslot address */
            if (!Storage_ExtFlash_ParaSec_Write(front_freeslot_addr, (uint8_t *)&FreeSlot_Info, sizeof(FreeSlot_Info)))
            {
                p_Sec->free_space_size = ori_freespace_size;
                return Storage_Write_Error;
            }

            /* write to current freeslot section */
            if (!Storage_ExtFlash_ParaSec_Write(slot_addr, (uint8_t *)slot_info, sizeof(Storage_FreeSlot_TypeDef)))
            {
                p_Sec->free_space_size = ori_freespace_size;
                return Storage_Write_Error;
            }

            p_Sec->free_slot_addr = slot_addr;
            p_Sec->free_space_size += sizeof(Storage_FreeSlot_TypeDef);
        }
        /* circumstance 2: new free slot is behind of the old free slot */
        else if (front_freeslot_addr + FreeSlot_Info.cur_slot_size == slot_addr)
        {
            /* merge behind free slot */
            FreeSlot_Info.cur_slot_size += slot_info->cur_slot_size + sizeof(Storage_FreeSlot_TypeDef);
            Storage_Assert(slot_info->nxt_addr < FreeSlot_Info.nxt_addr);
            FreeSlot_Info.nxt_addr = slot_info->nxt_addr;

            memset(slot_info, 0, sizeof(Storage_FreeSlot_TypeDef));

            /* write to new free slot */
            if (!Storage_ExtFlash_ParaSec_Write(slot_addr, (uint8_t *)slot_info, sizeof(Storage_FreeSlot_TypeDef)))
                return Storage_Write_Error;

            /* write to behind free slot */
            if (!Storage_ExtFlash_ParaSec_Write(front_freeslot_addr, (uint8_t *)&FreeSlot_Info, sizeof(Storage_FreeSlot_TypeDef)))
                return Storage_Write_Error;
        }
        /* circumstance 3: none free slot near by */
        else if (((front_freeslot_addr + FreeSlot_Info.cur_slot_size + sizeof(Storage_FreeSlot_TypeDef)) < slot_addr) && \
                 (behind_freeslot_addr > (slot_addr + slot_info->cur_slot_size + sizeof(Storage_FreeSlot_TypeDef))))
        {
            /* link free slot */
            if (Storage_Link_FreeSlot(front_freeslot_addr, behind_freeslot_addr, slot_addr, slot_info))
                return Storage_Error_None;

            return Storage_FreeSlot_Link_Error;
        }

        if (FreeSlot_Info.nxt_addr == 0)
            return Storage_Error_None;

        /* update front free slot address */
        front_freeslot_addr = behind_freeslot_addr;
    }

    return Storage_Delete_Error;
}

/* untested */
static bool Storage_DeleteSingleDataSlot(uint32_t slot_addr, uint8_t *p_data, Storage_BaseSecInfo_TypeDef *p_Sec)
{
    uint32_t cur_slot_size = 0;
    uint32_t inc_free_space = sizeof(Storage_DataSlot_TypeDef);
    uint8_t *p_freeslot_start = NULL;
    uint8_t *data_w = NULL;

    if ((slot_addr == 0) || \
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
         */
        *(uint32_t *)p_freeslot_start = 0;
        p_freeslot_start += sizeof(uint32_t);

        /* set end tag */
        *(uint32_t *)p_freeslot_start = STORAGE_SLOT_END_TAG;
    }
    else
        return false;

    /* update to data section */
    if (Storage_ExtFlash_ParaSec_Write(slot_addr, data_w, inc_free_space))
    {
        /* check free slot and merge */
        if (Storage_FreeSlot_CheckMerge(slot_addr, (Storage_FreeSlot_TypeDef *)p_freeslot_start, p_Sec) == Storage_Error_None)
            return true;
    }

    return false;
}

/* developping & untested */
static bool Storage_DeleteAllDataSlot(uint32_t addr, char *name, uint32_t total_size, Storage_BaseSecInfo_TypeDef *p_Sec)
{
    Storage_DataSlot_TypeDef data_slot;
    uint8_t *p_read = page_data_tmp;
    uint8_t name_len = 0;

    if ((addr == 0) || \
        (name == NULL) || \
        (strlen(name) == 0) || \
        (total_size == 0) || \
        (p_Sec == NULL))
        return false;

    memset(&data_slot, 0, sizeof(data_slot));
    name_len = strlen(name);

    if (!Storage_ExtFlash_ParaSec_Read(addr, page_data_tmp, total_size))
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
        if (!Storage_DeleteAllDataSlot(data_slot.nxt_addr, name, total_size, p_Sec))
            return false;
    }

    /* reset data slot as free slot */
    if (!Storage_DeleteSingleDataSlot(addr, page_data_tmp, p_Sec))
        return false;

    return true;
}

/* developping */
static Storage_ErrorCode_List Storage_DeleteItem(Storage_ParaClassType_List _class, const char *name, uint32_t size)
{
    Storage_FlashInfo_TypeDef *p_Flash = NULL;
    Storage_BaseSecInfo_TypeDef *p_Sec = NULL;
    Storage_ItemSearchOut_TypeDef ItemSearch;
    
    memset(&ItemSearch, 0, sizeof(ItemSearch));

    if( !Storage_Monitor.init_state || \
        (name == NULL) || \
        (strlen(name) == 0) || \
        (strlen(name) >= STORAGE_NAME_LEN) || \
        (size == 0))
        return Storage_Param_Error;

    p_Flash = &Storage_Monitor.external_info;
    p_Sec = Storage_Get_SecInfo(p_Flash, _class);
    if ((p_Sec == NULL) || \
        (p_Sec->para_num == 0) || \
        (p_Sec->para_size == 0))
        return Storage_Class_Error;

    /* search tab for item slot first */
    ItemSearch = Storage_Search(_class, name);
    if ((ItemSearch.item_addr == 0) || \
        (ItemSearch.item.data_addr == 0) || \
        (ItemSearch.item.head_tag != STORAGE_ITEM_HEAD_TAG) || \
        (ItemSearch.item.end_tag != STORAGE_ITEM_END_TAG) || \
        !Storage_DeleteAllDataSlot(ItemSearch.item.data_addr, (char *)name, ItemSearch.item.len, p_Sec))
        return Storage_Delete_Error;

    /* update item slot tab */
    memset(ItemSearch.item.name, '\0', STORAGE_NAME_LEN);
    memcpy(ItemSearch.item.name, STORAGE_FREEITEM_NAME, strlen(STORAGE_FREEITEM_NAME));

    if (!Storage_Comput_ItemSlot_CRC(&ItemSearch.item))
        return Storage_ItemUpdate_Error;

    if (!Storage_ItemSlot_Update(ItemSearch.item_addr, ItemSearch.item_index, p_Sec, ItemSearch.item))
        return Storage_ItemUpdate_Error;

    /* update base info */

    return Storage_Delete_Error;
}

static Storage_ErrorCode_List Storage_CreateItem(Storage_ParaClassType_List _class, const char *name, uint8_t *p_data, uint16_t size)
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
    uint32_t slot_useful_size = 0;
    uint8_t item_index = 0;
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

    p_Flash = &Storage_Monitor.external_info;
    p_Sec = Storage_Get_SecInfo(p_Flash, _class);
    if (p_Sec == NULL)
        return Storage_Class_Error;

    if (p_Sec->free_slot_addr == 0)
        return Storage_FreeSlot_Addr_Error;

    if (Storage_ExtFlash_ParaSec_Read(p_Sec->free_slot_addr, (uint8_t *)&FreeSlot, sizeof(Storage_FreeSlot_TypeDef)) && \
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
            if (!Storage_ExtFlash_ParaSec_Read(storage_tab_addr, page_data_tmp, (p_Sec->tab_size / p_Sec->page_num)))
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
                    crt_item_slot._class = _class;
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

                p_data += stored_size;
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
                    if (!Storage_ExtFlash_ParaSec_Read(cur_freeslot_addr, (uint8_t *)&FreeSlot, sizeof(Storage_FreeSlot_TypeDef)))
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
                crc_buf = slot_update_ptr;
                memcpy(slot_update_ptr, p_data, (DataSlot.cur_slot_size - DataSlot.align_size));
                slot_update_ptr += (DataSlot.cur_slot_size - DataSlot.align_size);
                
                if (DataSlot.align_size)
                    memset(slot_update_ptr, 0, DataSlot.align_size);
                
                slot_update_ptr += DataSlot.align_size;

                /* comput current slot crc */
                DataSlot.slot_crc = Common_CRC16(crc_buf, DataSlot.cur_slot_size);
                memcpy(slot_update_ptr, &DataSlot.slot_crc, sizeof(DataSlot.slot_crc));
                slot_update_ptr += sizeof(DataSlot.slot_crc);
                memcpy(slot_update_ptr, &DataSlot.end_tag, sizeof(DataSlot.end_tag));
                slot_update_ptr += sizeof(DataSlot.end_tag);

                /* step 3: store data to data section */
                if (!Storage_ExtFlash_ParaSec_Write(store_addr, page_data_tmp, (DataSlot.cur_slot_size + sizeof(DataSlot))))
                    return Storage_Write_Error;

                if (DataSlot.nxt_addr == 0)
                {
                    if (DataSlot.total_data_size == stored_size)
                    {
                        /* step 4: update free slot */
                        if (!Storage_ExtFlash_ParaSec_Write(cur_freeslot_addr, (uint8_t *)&FreeSlot, sizeof(FreeSlot)))
                            return Storage_Write_Error;

                        break;
                    }

                    return Storage_No_Enough_Space;
                }
                else
                {
                    /* after target data segment stored, shift target data pointer to unstored pos
                        * and update next segment data store address */
                    stored_size += slot_useful_size;
                    store_addr = DataSlot.nxt_addr;
                }
            }
        }

        /* get tab */
        if (!Storage_ExtFlash_ParaSec_Read(storage_tab_addr, page_data_tmp, (p_Sec->tab_size / p_Sec->page_num)))
            return Storage_Read_Error;

        tab_item = (Storage_Item_TypeDef *)page_data_tmp;
        tab_item[item_index] = crt_item_slot;

        /* write back item slot list to tab */
        if (!Storage_ExtFlash_ParaSec_Write(storage_tab_addr, page_data_tmp, (p_Sec->tab_size / p_Sec->page_num)))
            return Storage_Write_Error;

        /* update free slot address in base info */
        p_Sec->free_slot_addr = cur_freeslot_addr;

        /* update base info crc */
        memcpy(page_data_tmp, p_Flash, sizeof(Storage_FlashInfo_TypeDef));
        base_info_crc = Common_CRC16(page_data_tmp, Storage_InfoPageSize - sizeof(base_info_crc));
        memcpy(&page_data_tmp[Storage_InfoPageSize - sizeof(base_info_crc)], &base_info_crc, sizeof(base_info_crc));
        
        /* update base info from section start*/
        if (!Storage_ExtFlash_ParaSec_Write(From_Start_Address, page_data_tmp, Storage_InfoPageSize))
            return Storage_Write_Error;
    }

    return Storage_Error_None;
}

static bool Storage_Establish_Tab(Storage_ParaClassType_List class)
{
    Storage_FlashInfo_TypeDef *p_Flash = NULL;
    Storage_BaseSecInfo_TypeDef *p_SecInfo = NULL;
    uint16_t clear_cnt = 0;
    uint32_t clear_byte = 0;
    uint32_t clear_remain = 0;
    uint32_t addr_tmp = 0;
    Storage_FreeSlot_TypeDef free_slot;
    uint16_t crc = 0;

    p_Flash = &Storage_Monitor.external_info;
    p_SecInfo = Storage_Get_SecInfo(p_Flash, class);
    if (p_SecInfo == NULL)
        return false;

    if (p_SecInfo->tab_addr && Storage_Clear_Tab(p_SecInfo->tab_addr, p_SecInfo->page_num))
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
            if (!Storage_ExtFlash_ParaSec_Write(addr_tmp, page_data_tmp, clear_byte))
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
        if (!Storage_ExtFlash_ParaSec_Write(p_SecInfo->data_sec_addr, page_data_tmp, Storage_TabSize))
            return false;

        /* update info section */
        p_SecInfo->free_slot_addr = p_SecInfo->data_sec_addr;
        p_SecInfo->free_space_size = p_SecInfo->data_sec_size;
        p_SecInfo->para_num = 0;
        p_SecInfo->para_size = 0;

        /* read out whole section info data from storage info section */
        if (!Storage_ExtFlash_ParaSec_Read(From_Start_Address, page_data_tmp, Storage_TabSize))
            return false;

        memset(page_data_tmp, 0, Storage_InfoPageSize);
        memcpy(page_data_tmp, p_Flash, sizeof(Storage_FlashInfo_TypeDef));

        /* comput crc */
        crc = Common_CRC16(page_data_tmp, Storage_InfoPageSize - sizeof(crc));
        memcpy(&page_data_tmp[Storage_InfoPageSize - sizeof(crc)], &crc, sizeof(crc));

        /* erase sector first then write into the target sector */
        if (!Storage_ExtFlash_ParaSec_Write(From_Start_Address, page_data_tmp, Storage_TabSize))
            return false;

        return true;
    }

    return false;
}

static bool Storage_Build_StorageInfo(void)
{
    uint32_t page_num = 0;
    Storage_FlashInfo_TypeDef Info;
    Storage_FlashInfo_TypeDef Info_Rx;
    uint32_t addr_offset = 0;
    uint32_t BaseInfo_start_addr = 0;
    uint32_t tab_addr_offset = 0;
    uint16_t crc = 0;
    uint32_t remain_data_sec_size = 0;
    uint32_t data_sec_size = 0;

    memset(&Info, 0, sizeof(Storage_FlashInfo_TypeDef));
    memset(&Info_Rx, 0, sizeof(Storage_FlashInfo_TypeDef));
    memcpy(Info.tag, EXTERNAL_STORAGE_PAGE_TAG, EXTERNAL_PAGE_TAG_SIZE);

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

    /* write 0 to info section */
    memset(page_data_tmp, 0, Storage_InfoPageSize);

    /* read out and erase sector */
    if (!Storage_ExtFlash_ParaSec_Read(addr_offset, page_data_tmp, sizeof(Info)))
        return false;

    /* write base info to info section */
    memcpy(page_data_tmp, &Info, sizeof(Info));
    crc = Common_CRC16(page_data_tmp, Storage_InfoPageSize - sizeof(crc));
    memcpy(&page_data_tmp[Storage_InfoPageSize - sizeof(crc)], &crc, sizeof(crc));

    /* write into flash chip */
    if (!Storage_ExtFlash_ParaSec_Write(addr_offset, page_data_tmp, Storage_InfoPageSize))
        return false;

    /* read out again */
    if (!Storage_ExtFlash_ParaSec_Read(addr_offset, page_data_tmp, sizeof(Info)))
        return false;

    /* compare with target */
    memcpy(&Info_Rx, page_data_tmp, sizeof(Info_Rx));
    if (memcmp(&Info_Rx, &Info, sizeof(Storage_FlashInfo_TypeDef)) != 0)
        return false;

    if (!Storage_Establish_Tab(Para_Boot) || \
        !Storage_Establish_Tab(Para_Sys)  || \
        !Storage_Establish_Tab(Para_User))
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

/********************************************** BlackBox Storage API Section *****************************************************/
static bool Storage_Write_Section(uint32_t addr, uint8_t *p_data, uint16_t len)
{
    uint32_t write_cnt = 0;
    uint32_t addr_tmp = 0;
    Storage_ExtFLashDevObj_TypeDef *p_dev = NULL;

    p_dev = (Storage_ExtFLashDevObj_TypeDef *)Storage_Monitor.ExtDev_ptr;
    if (addr && p_data && len && p_dev && p_dev->dev_api && p_dev->dev_obj)
    {
        if ((addr % p_dev->sector_size) || \
            (len % p_dev->sector_size))
            return false;

        write_cnt = len / p_dev->sector_size;
        addr_tmp = addr;

        for (uint8_t i = 0; i < write_cnt; i ++)
        {
            switch((uint8_t)p_dev->chip_type)
            {
                case Storage_ChipType_W25Qxx:
                    /* erase sector */
                    if (To_DevW25Qxx_API(p_dev->dev_api)->erase_sector(To_DevW25Qxx_OBJ(p_dev->dev_obj), addr_tmp) != DevW25Qxx_Ok)
                    {
                        STORAGE_INFO("section erase failed\r\n");
                        return false;
                    }

                    /* update sector */
                    if (To_DevW25Qxx_API(p_dev->dev_api)->write(To_DevW25Qxx_OBJ(p_dev->dev_obj), addr_tmp, p_data, p_dev->sector_size))
                    {
                        STORAGE_INFO("section write failed\r\n");
                        return false;
                    }
                    break;

                default: return false;
            }

            addr_tmp += p_dev->sector_size;
        }

        return true;
    }

    return false;
}

static bool Storage_Read_Section(uint32_t addr, uint8_t *p_data, uint16_t len)
{
    uint32_t read_cnt = 0;
    uint32_t addr_tmp = 0;
    Storage_ExtFLashDevObj_TypeDef *p_dev = NULL;

    p_dev = (Storage_ExtFLashDevObj_TypeDef *)Storage_Monitor.ExtDev_ptr;
    if (addr && p_data && len && p_dev && p_dev->dev_api && p_dev->dev_obj)
    {
        if ((addr % p_dev->sector_size) || \
            (len % p_dev->sector_size))
            return false;
    
        read_cnt = len / p_dev->sector_size;
        addr_tmp = addr;

        for (uint8_t i = 0; i < read_cnt; i++)
        {
            switch((uint8_t)p_dev->chip_type)
            {
                case Storage_ChipType_W25Qxx:
                    /* read sector */
                    if (To_DevW25Qxx_API(p_dev->dev_api)->read(To_DevW25Qxx_OBJ(p_dev->dev_obj), addr_tmp, p_data, len) != DevW25Qxx_Ok)
                    {
                        STORAGE_INFO("section read failed\r\n");
                        memset(p_data, 0, len);
                        return false;
                    }
                    break;

                default: return false;
            }

            addr_tmp += p_dev->sector_size;
        }

        return true;
    }

    return false;
}

static bool Storage_Erase_Section(uint32_t addr, uint16_t len)
{
    uint32_t erase_cnt = 0;
    uint32_t addr_tmp = 0;
    Storage_ExtFLashDevObj_TypeDef *p_dev = NULL;

    p_dev = (Storage_ExtFLashDevObj_TypeDef *)Storage_Monitor.ExtDev_ptr;
    if (addr && len && p_dev && p_dev->dev_api && p_dev->dev_obj)
    {
        if ((addr % p_dev->sector_size) || \
            (len % p_dev->sector_size))
            return false;
    
        erase_cnt = len / p_dev->sector_size;
        addr_tmp = addr;

        for (uint8_t i = 0; i < erase_cnt; i ++)
        {
            switch((uint8_t)p_dev->chip_type)
            {
                case Storage_ChipType_W25Qxx:
                    /* erase sector */
                    if (To_DevW25Qxx_API(p_dev->dev_api)->erase_sector(To_DevW25Qxx_OBJ(p_dev->dev_obj), addr_tmp) != DevW25Qxx_Ok)
                        return false;
                    break;

                default: return false;
            }

            addr_tmp += p_dev->sector_size;
        }

        return true;
    }

    return false;
}

/********************************************** External Firmware Storage API Section ********************************************/
static bool Storage_Firmware_Format(Storage_FirmwareType_List type)
{
    uint32_t format_size = 0;
    uint32_t erase_addr = 0;
    Storage_ExtFLashDevObj_TypeDef *dev = NULL;

    if (Storage_Monitor.init_state && (type != Firmware_None) && (type <= Firmware_App))
    {
        dev = (Storage_ExtFLashDevObj_TypeDef *)(Storage_Monitor.ExtDev_ptr);
        if (dev == NULL)
            return false;

        switch ((uint8_t)type)
        {
            case Firmware_App:
                format_size = App_Firmware_Size;
                erase_addr = App_Firmware_Addr;
                break;

            case Firmware_Boot:
                format_size = Boot_Firmware_Size;
                erase_addr = Boot_Firmware_Addr;
                break;

            default: break;
        }

        if ((format_size % Storage_TabSize) != 0)
            return false;

        for (uint16_t i = 0; i < format_size / Storage_TabSize; i++)
        {
            switch (dev->chip_type)
            {
                case Storage_ChipType_W25Qxx:
                    if (dev->dev_api && dev->dev_obj)
                    {
                        if (format_size == 0)
                            return true;

                        if (To_DevW25Qxx_API(dev->dev_api)->erase_sector(To_DevW25Qxx_OBJ(dev->dev_obj), erase_addr) != DevW25Qxx_Ok)
                            return false;
                    
                        erase_addr += Storage_TabSize;
                        format_size -= Storage_TabSize;
                    }
                    break;

                default: break;
            }
        }
    }

    return false;
}

static bool Storage_Frimware_Read(Storage_FirmwareType_List type, uint32_t addr_offset, uint8_t *p_data, uint16_t size)
{
    uint32_t base_addr = 0;
    uint32_t firmware_size = 0;
    uint32_t read_addr = 0;
    uint32_t section_addr = 0;
    uint32_t read_size = 0;
    Storage_ExtFLashDevObj_TypeDef *dev = NULL;
    dev = (Storage_ExtFLashDevObj_TypeDef *)(Storage_Monitor.ExtDev_ptr);

    if (dev && p_data && size)
    {
        switch ((uint8_t)type)
        {
            case Firmware_App:
                base_addr = App_Firmware_Addr;
                firmware_size = App_Firmware_Size;
                break;

            case Firmware_Boot:
                base_addr = Boot_Firmware_Addr;
                firmware_size = Boot_Firmware_Size;
                break;

            default: return false;
        }
        
        read_addr = addr_offset + base_addr;
        while (true)
        {
            section_addr = To_DevW25Qxx_API(dev->dev_api)->get_section_start_addr(To_DevW25Qxx_OBJ(dev->dev_obj), read_addr);
            if (To_DevW25Qxx_API(dev->dev_api)->read(To_DevW25Qxx_OBJ(dev->dev_obj), section_addr, flash_read_tmp, Storage_TabSize) != DevW25Qxx_Ok)
                return false;

            if ((read_addr + size) > (section_addr + Storage_TabSize))
            {
                read_size = Storage_TabSize - (read_addr - section_addr);
                size -= read_size;
                p_data += read_size;
            }
            else
            {
                read_size = size;
                size = 0;
            }

            memcpy(p_data, &flash_read_tmp[read_addr - section_addr], read_size);
            read_addr = section_addr + Storage_TabSize;

            if (size == 0)
                return true;
        }
    }

    return false;
}

static bool Storage_Firmware_Write(Storage_MediumType_List medium, Storage_FirmwareType_List type, uint32_t addr_offset, uint8_t *p_data, uint16_t size)
{
    uint32_t ext_base_addr = 0;
    uint32_t int_base_addr = 0;
    uint32_t write_addr = 0;
    uint32_t section_addr = 0;
    uint32_t write_size = 0;
    Storage_ExtFLashDevObj_TypeDef *dev = NULL;

    if (p_data && size)
    {
        switch ((uint8_t)type)
        {
            case Firmware_Boot:
                ext_base_addr = Boot_Firmware_Addr;
                int_base_addr = Boot_Address_Base;
                break;

            case Firmware_App:
                ext_base_addr = App_Firmware_Addr;
                int_base_addr = App_Address_Base;
                break;

            default: return false;
        }

        if (medium == Internal_Flash)
        {
            write_addr = int_base_addr + addr_offset;
            BspFlash.write(write_addr, p_data, size);
        }
        else if (medium == External_Flash)
        {
            dev = (Storage_ExtFLashDevObj_TypeDef *)(Storage_Monitor.ExtDev_ptr);
            if (dev == NULL)
                return false;

            switch ((uint8_t)dev->chip_type)
            {
                case Storage_ChipType_W25Qxx:
                    write_addr = ext_base_addr + addr_offset;
                    section_addr = To_DevW25Qxx_API(dev->dev_api)->get_section_start_addr(To_DevW25Qxx_OBJ(dev->dev_obj), write_addr);

                    while (true)
                    {
                        if (size == 0)
                            return true;

                        /* read section first */
                        memset(flash_read_tmp, 0, Storage_TabSize);
                        if (To_DevW25Qxx_API(dev->dev_api)->read(To_DevW25Qxx_OBJ(dev->dev_obj), section_addr, flash_read_tmp, Storage_TabSize) != DevW25Qxx_Ok)
                            return false;

                        /* erase whole section */
                        if (To_DevW25Qxx_API(dev->dev_api)->erase_sector(To_DevW25Qxx_OBJ(dev->dev_obj), section_addr) != DevW25Qxx_Ok)
                            return false;

                        if ((write_addr + size) >= (section_addr + Storage_TabSize))
                        {
                            write_size = Storage_TabSize - (write_addr - section_addr);
                            size -= write_size;
                        }
                        else
                        {
                            write_size = size;
                            size = 0;
                        }

                        /* update to flash */
                        memcpy(&flash_read_tmp[write_addr - section_addr], p_data, write_size);
                        if (To_DevW25Qxx_API(dev->dev_api)->write(To_DevW25Qxx_OBJ(dev->dev_obj), section_addr, flash_read_tmp, Storage_TabSize) != DevW25Qxx_Ok)
                            return false;

                        /* update section address */
                        p_data += write_size;
                        write_addr += write_size;
                        section_addr = To_DevW25Qxx_API(dev->dev_api)->get_section_start_addr(To_DevW25Qxx_OBJ(dev->dev_obj), write_addr);
                    }
                    break;

                default: return false;
            }
        }
    }

    return false;
}

/************************************************** External Flash IO API Section ************************************************/
static bool Storage_External_Chip_W25Qxx_SelectPin_Ctl(bool state)
{
#if (FLASH_CHIP_STATE == ON)
    BspGPIO.write(ExtFlash_CS_Pin, state);
    return true;
#else
    return false;
#endif
}

static uint16_t Storage_External_Chip_W25Qxx_BusTx(uint8_t *p_data, uint16_t len, uint32_t time_out)
{
#if (FLASH_CHIP_STATE == ON)
    BspSPI_NorModeConfig_TypeDef *p_cfg = To_NormalSPI_ObjPtr(Storage_Monitor.ExtBusCfg_Ptr);

    if (p_data && len && p_cfg && p_cfg->Instance && ExtFlash_Bus_InstObj)
    {
        if (ExtFlash_Bus_Api.trans(&ExtFlash_Bus_InstObj, p_data, len, time_out))
            return len;
    }
#endif

    return 0;
}

static uint16_t Storage_External_Chip_W25Qxx_BusRx(uint8_t *p_data, uint16_t len, uint32_t time_out)
{
    BspSPI_NorModeConfig_TypeDef *p_cfg = To_NormalSPI_ObjPtr(Storage_Monitor.ExtBusCfg_Ptr);

#if (FLASH_CHIP_STATE == ON)
    if (p_data && len && p_cfg && p_cfg->Instance && ExtFlash_Bus_InstObj)
    {
        if (ExtFlash_Bus_Api.receive(&ExtFlash_Bus_InstObj, p_data, len, time_out))
            return len;
    }
#endif

    return 0;
}

static uint16_t Storage_External_Chip_W25Qxx_BusTrans(uint8_t *tx, uint8_t *rx, uint16_t len, uint32_t time_out)
{
    BspSPI_NorModeConfig_TypeDef *p_cfg = To_NormalSPI_ObjPtr(Storage_Monitor.ExtBusCfg_Ptr);

#if (FLASH_CHIP_STATE == ON)
    if (tx && rx && len && p_cfg && p_cfg->Instance && ExtFlash_Bus_InstObj)
    {
        if (ExtFlash_Bus_Api.trans_receive(&ExtFlash_Bus_InstObj, tx, rx, len, time_out))
            return len;
    }
#endif

    return 0;
}

/************************************************** External Flash Parameter IO API Section ************************************************/
static bool Storage_ExtFlash_ParaSec_Read(uint32_t addr_offset, uint8_t *p_data, uint32_t len)
{
    uint32_t read_start_addr = 0;
    uint32_t flash_end_addr = 0;
    uint32_t section_start_addr = 0;
    uint32_t next_read_addr = 0;
    uint32_t section_size = 0;
    uint32_t read_offset = 0;
    uint32_t read_len = len;
    Storage_ExtFLashDevObj_TypeDef *dev = NULL;

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

static bool Storage_ExtFlash_ParaSec_Write(uint32_t addr_offset, uint8_t *p_data, uint32_t len)
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

    return false;
}

static bool Storage_ExtFlash_ParaSec_Erase(uint32_t addr_offset, uint32_t len)
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

#if (STORAGE_DEBUG == 1)
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

        case Storage_FreeSlot_Info_Error:
            return Storage_ErrorCode_ToStr(Storage_FreeSlot_Info_Error);

        case Storage_FreeSlot_Link_Error:
            return Storage_ErrorCode_ToStr(Storage_FreeSlot_Link_Error);

        case Storage_ItemInfo_Error:
            return Storage_ErrorCode_ToStr(Storage_ItemInfo_Error);
            
        case Storage_ItemUpdate_Error:
            return Storage_ErrorCode_ToStr(Storage_ItemUpdate_Error);

        case Storage_CRC_Error:
            return Storage_ErrorCode_ToStr(Storage_CRC_Error);
            
        case Storage_Update_DataSize_Error:
            return Storage_ErrorCode_ToStr(Storage_Update_DataSize_Error);
            
        case Storage_Delete_Error:
            return Storage_ErrorCode_ToStr(Storage_Delete_Error);

        default:
            return "Unknow Error\r\n";
    }
}

static bool Storage_Get_Flash_Section_IOAPI(Shell *shell_obj, \
                                            Storage_ParaClassType_List class, \
                                            Storage_FlashInfo_TypeDef **p_Flash, \
                                            Storage_BaseSecInfo_TypeDef **p_Sec)
{
    if (shell_obj == NULL)
        return false;

    if (!Storage_Monitor.init_state)
    {
        shellPrint(shell_obj, "\t[External_Flash Unavaliable]\r\n");
        shellPrint(shell_obj, "\t[Format cnt   : %d]\r\n", Storage_Monitor.ExternalFlash_Format_cnt);
        shellPrint(shell_obj, "\t[Buid Tab cnt : %d]\r\n", Storage_Monitor.ExternalFlash_BuildTab_cnt);
        return false;
    }

    *p_Flash = &Storage_Monitor.external_info;
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

    return true;
}

static void Storage_Shell_Get_BaseInfo(void)
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
    shellPrint(shell_obj, "\t\t[External_Flash Selected]\r\n");
    p_Flash = &Storage_Monitor.external_info;

    if(memcmp(p_Flash->tag, EXTERNAL_STORAGE_PAGE_TAG, EXTERNAL_PAGE_TAG_SIZE))
    {
        shellPrint(shell_obj, "\t\tExternal_Flash Info Error\r\n");
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

static void Storage_Test(Storage_ParaClassType_List _class, char *test_name, char *test_data)
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
    
    shellPrint(shell_obj, "\t[External_Flash Selected]\r\n");
    shellPrint(shell_obj, "\t[External_Flash Unavaliable]\r\n");
    shellPrint(shell_obj, "\t[Format cnt   : %d]\r\n", Storage_Monitor.ExternalFlash_Format_cnt);
    shellPrint(shell_obj, "\t[Buid Tab cnt : %d]\r\n", Storage_Monitor.ExternalFlash_BuildTab_cnt);
    shellPrint(shell_obj, "\thalt by enable or init state\r\n");

    Storage_ClassType_Print(shell_obj);
    if(_class > Para_User)
    {
        shellPrint(shell_obj, "\tstorage class arg error\r\n");
        shellPrint(shell_obj, "\ttest halt\r\n");
        return;
    }

    Storage_SelectedClass_Print(shell_obj, _class);
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
    if (Storage_Search(_class, test_name).item.data_addr != 0)
    {
        shellPrint(shell_obj, "\t%s already exist\r\n", test_name);
        return;
    }

    error_code = Storage_CreateItem(_class, test_name, (uint8_t *)test_data, strlen(test_data));
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

static void Storage_Module_Format(void)
{
    Shell *shell_obj = Shell_GetInstence();
    
    if ((shell_obj == NULL) || !Storage_Monitor.init_state)
    {
        shellPrint(shell_obj, "\t[External_Flash Unavaliable]\r\n");
        return;
    }

    shellPrint(shell_obj, "\t[Flash formatting ...]\r\n");
    if (Storage_Format())
    {
        shellPrint(shell_obj, "\t[Flash formatting done]\r\n");
        shellPrint(shell_obj, "\t[Rebuilding storage tab and section]\r\n");

        /* rebuild tab */
        if (!Storage_Build_StorageInfo())
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
    uint32_t FreeSlot_addr = 0;
    Shell *shell_obj = Shell_GetInstence();

    if (shell_obj == NULL)
        return;

    shellPrint(shell_obj, "[Storage Shell]Show Freeslot\r\n");
    if(!Storage_Monitor.init_state)
    {
        shellPrint(shell_obj, "\thalt by init state\r\n");
    }

    if (!Storage_Get_Flash_Section_IOAPI(shell_obj, class, &p_Flash, &p_Sec))
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
        if (!Storage_ExtFlash_ParaSec_Read(FreeSlot_addr, page_data_tmp, sizeof(Storage_FreeSlot_TypeDef)))
        {
            shellPrint(shell_obj, "\t[FreeSlot data read error]\r\n");
            return;
        }

        p_FreeSlot = (Storage_FreeSlot_TypeDef *)page_data_tmp;
        
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
    Storage_DataSlot_TypeDef DataSlot;
    Storage_ItemSearchOut_TypeDef ItemSearch;
    uint8_t *crc_buf = NULL;
    uint16_t crc = 0;
    uint16_t crc_len = 0;
    uint16_t data_len = 0;
    uint32_t data_addr = 0;
    Shell *shell_obj = Shell_GetInstence();
    uint8_t *p_read_out = NULL;
    uint16_t i = 0;

    memset(&ItemSearch, 0, sizeof(ItemSearch));
    memset(&DataSlot, 0, sizeof(DataSlot));

    if ((shell_obj == NULL) || \
        (name == NULL) || \
        (strlen((char *)name) == 0))
        return;
    
    if (!Storage_Get_Flash_Section_IOAPI(shell_obj, class, &p_Flash, &p_Sec))
    {
        shellPrint(shell_obj, "\t[Flash Section IO_API Get Error]\r\n");
        return;
    }

    ItemSearch = Storage_Search(class, (const char *)name);
    if (ItemSearch.item.data_addr)
    {
        shellPrint(shell_obj, "\t[tab item %s matched]\r\n", name);
        shellPrint(shell_obj, "\t[data size : %d]\r\n", ItemSearch.item.len);
        shellPrint(shell_obj, "\t[data name : %s]\r\n", ItemSearch.item.name);
        shellPrint(shell_obj, "\t\r\n");

        data_len = ItemSearch.item.len;
        data_addr = ItemSearch.item.data_addr;

        while(data_len)
        {
            if (!Storage_ExtFlash_ParaSec_Read(data_addr, page_data_tmp, data_len + sizeof(Storage_DataSlot_TypeDef)))
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
    
    if (!Storage_Get_Flash_Section_IOAPI(shell_obj, class, &p_Flash, &p_Sec))
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

        if (!Storage_ExtFlash_ParaSec_Read(tab_addr, page_data_tmp, singal_tab_size))
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
            item_list = (Storage_Item_TypeDef *)page_data_tmp;
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

                    shellPrint(shell_obj, "\t\t[item class    : %d]\r\n", item_list[j]._class);
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
    if (!Storage_ExtFlash_ParaSec_Read(p_Sec->free_slot_addr, page_data_tmp, sizeof( Storage_FreeSlot_TypeDef)))
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

static void Storage_Dump_DataSection(Storage_ParaClassType_List class)
{
    uint32_t flash_sector_size = 0;
    uint32_t remain_dump_size = 0;
    uint32_t dump_size = 0;
    uint32_t dump_addr = 0;
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

    if (!Storage_Get_Flash_Section_IOAPI(shell_obj, class, &p_Flash, &p_Sec))
    {
        shellPrint(shell_obj, "\t[Flash Section IO_API Get Error]\r\n");
        return;
    }

    if (Storage_Monitor.ExtDev_ptr)
    {
        ext_dev = (Storage_ExtFLashDevObj_TypeDef *)Storage_Monitor.ExtDev_ptr;
        if (ext_dev->chip_type == Storage_ChipType_W25Qxx)
        {
            flash_sector_size = To_DevW25Qxx_API(ext_dev->dev_api)->info(ext_dev->dev_obj).subsector_size;
        }
    }

    remain_dump_size = p_Sec->data_sec_size;
    dump_size = remain_dump_size;
    dump_addr = p_Sec->data_sec_addr;
    while(remain_dump_size)
    {
        if (dump_size >= flash_sector_size)
            dump_size = flash_sector_size;

        if (!Storage_ExtFlash_ParaSec_Read(dump_addr, page_data_tmp, dump_size))
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
    Storage_FlashInfo_TypeDef *p_Flash = NULL;
    Storage_BaseSecInfo_TypeDef *p_Sec = NULL;
    Storage_ItemSearchOut_TypeDef ItemSearch;
    Storage_ErrorCode_List update_error_code = Storage_Error_None;
    
    memset(&ItemSearch, 0, sizeof(ItemSearch));

    if ((shell_obj == NULL) || \
        (test_name == NULL) || \
        (test_data == NULL) || \
        (strlen(test_name) == 0) || \
        (strlen(test_data) == 0))
        return;

    if (!Storage_Get_Flash_Section_IOAPI(shell_obj, class, &p_Flash, &p_Sec))
    {
        shellPrint(shell_obj, "\t[Flash Section IO_API Get Error]\r\n");
        return;
    }

    ItemSearch = Storage_Search(class, test_name);
    if ((ItemSearch.item.head_tag != STORAGE_ITEM_HEAD_TAG) || \
        (ItemSearch.item.end_tag != STORAGE_ITEM_END_TAG) || \
        (ItemSearch.item.data_addr == 0))
    {
        shellPrint(shell_obj, "\t[Item slot error]\r\n");
        return;
    }

    update_error_code = Storage_SlotData_Update(class, ItemSearch.item.data_addr, (uint8_t *)test_data, strlen(test_data)); 
    if (update_error_code != Storage_Error_None)
    {
        shellPrint(shell_obj, "\t[Data update failed %s]\r\n", Storage_Error_Print(update_error_code));
        return;
    }
    else
        shellPrint(shell_obj, "\t[DataSlot Update accomplished]\r\n");
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, Storage_UpdateData, Storage_UpdateData, Storage update data);

/* untested */
static void Storage_DeleteData_Test(Storage_ParaClassType_List class, char *test_name)
{
    Shell *shell_obj = Shell_GetInstence();
    Storage_FlashInfo_TypeDef *p_Flash = NULL;
    Storage_BaseSecInfo_TypeDef *p_Sec = NULL;
    Storage_ItemSearchOut_TypeDef ItemSearch;
    
    if (shell_obj == NULL)
        return;
    
    if (!Storage_Get_Flash_Section_IOAPI(shell_obj, class, &p_Flash, &p_Sec))
    {
        shellPrint(shell_obj, "\t[Flash Section IO_API Get Error]\r\n");
        return;
    }

    /* search */
    ItemSearch = Storage_Search(class, test_name);
    if ((ItemSearch.item.head_tag != STORAGE_ITEM_HEAD_TAG) || \
        (ItemSearch.item.end_tag != STORAGE_ITEM_END_TAG) || \
        (ItemSearch.item.data_addr == 0))
    {
        shellPrint(shell_obj, "\t[Item slot error]\r\n");
        return;
    }

    if (!Storage_DeleteAllDataSlot(ItemSearch.item.data_addr, (char *)ItemSearch.item.name, ItemSearch.item.len, p_Sec))
    {
        shellPrint(shell_obj, "\t[Item Delete Error]\r\n");
    }
    else
        shellPrint(shell_obj, "\t[Item Delete Accomplished]\r\n");
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, Storage_DeleteData, Storage_DeleteData_Test, Storage delete data);
#endif

