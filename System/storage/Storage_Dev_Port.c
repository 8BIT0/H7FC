#include "Storage_Dev_Port.h"
#include "Srv_OsCommon.h"
#include "Dev_W25Qxx.h"
#include "Dev_W25Nxx.h"

#define Storage_Dev_GetSstsTick SrvOsCommon.get_os_ms
#define Storage_Dev_Malloc(x)   SrvOsCommon.malloc(x)
#define Storage_Dev_Free(x)     SrvOsCommon.free(x)

/* external function */
static bool Storage_Dev_Set(StorageDevObj_TypeDef *ext_dev);
static bool Storage_Dev_Init(StorageDevObj_TypeDef *ext_dev, uint16_t *p_type, uint16_t *p_code);
static bool Storage_Dev_Write_Section(StorageDevObj_TypeDef *p_dev, uint32_t addr, uint8_t *p_data, uint16_t len);
static bool Storage_Dev_Read_Section(StorageDevObj_TypeDef *p_dev, uint32_t addr, uint8_t *p_data, uint16_t len);
static bool Storage_Dev_Erase_Section(StorageDevObj_TypeDef *p_dev, uint32_t addr, uint16_t len);

StorageDevApi_TypeDef StorageDev = {
    .set = Storage_Dev_Set,
    .init = Storage_Dev_Init,

    .write_sec = Storage_Dev_Write_Section,
    .read_sec = Storage_Dev_Read_Section,
    .erase_sec = Storage_Dev_Erase_Section,
};

static bool Storage_Dev_Set(StorageDevObj_TypeDef *ext_dev)
{
    if ((ext_dev == NULL) || \
        (ext_dev->api == NULL))
        return false;

    ext_dev->start_addr  = 0;
    ext_dev->sector_num  = 0;
    ext_dev->sector_size = 0;
    ext_dev->total_size  = 0;
    ext_dev->page_num    = 0;
    ext_dev->page_size   = 0;

    if (ext_dev->chip_type == Storage_ChipType_W25Qxx)
    {
        ext_dev->obj = Storage_Dev_Malloc(sizeof(DevW25QxxObj_TypeDef));
        if ((ext_dev->obj == NULL) || \
            (To_DevW25Qxx_API(ext_dev->api)->info == NULL))
            return false;

        ext_dev->start_addr  = W25QXX_BASE_ADDRESS;
        ext_dev->sector_num  = To_DevW25Qxx_API(ext_dev->api)->info(To_DevW25Qxx_OBJ(ext_dev->obj)).subsector_num;
        ext_dev->sector_size = To_DevW25Qxx_API(ext_dev->api)->info(To_DevW25Qxx_OBJ(ext_dev->obj)).subsector_size;
        ext_dev->total_size  = To_DevW25Qxx_API(ext_dev->api)->info(To_DevW25Qxx_OBJ(ext_dev->obj)).flash_size;
        ext_dev->page_num    = To_DevW25Qxx_API(ext_dev->api)->info(To_DevW25Qxx_OBJ(ext_dev->obj)).page_num;
        ext_dev->page_size   = To_DevW25Qxx_API(ext_dev->api)->info(To_DevW25Qxx_OBJ(ext_dev->obj)).page_size;

        To_DevW25Qxx_OBJ(ext_dev->obj)->systick   = Storage_Dev_GetSstsTick;
        To_DevW25Qxx_OBJ(ext_dev->obj)->cs_ctl    = StoragePort_Api.cs_ctl;
        To_DevW25Qxx_OBJ(ext_dev->obj)->bus_tx    = StoragePort_Api.bus_tx;
        To_DevW25Qxx_OBJ(ext_dev->obj)->bus_rx    = StoragePort_Api.bus_rx;
        To_DevW25Qxx_OBJ(ext_dev->obj)->bus_trans = StoragePort_Api.bus_trans;
        To_DevW25Qxx_OBJ(ext_dev->obj)->delay_ms  = SrvOsCommon.delay_ms;

        return true;
    }
    else if (ext_dev->chip_type == Storage_ChipType_W25Nxx)
    {
        ext_dev->obj = Storage_Dev_Malloc(sizeof(DevW25NxxObj_TypeDef));
        if ((ext_dev->obj == NULL) || \
            (To_DevW25Nxx_API(ext_dev->api)->info == NULL))
            return false;
        
        ext_dev->start_addr  = W25NXX_BASE_ADDRESS;
        ext_dev->total_size  = To_DevW25Nxx_API(ext_dev->api)->info(To_DevW25Nxx_OBJ(ext_dev->obj)).flash_size;
        ext_dev->page_num    = To_DevW25Nxx_API(ext_dev->api)->info(To_DevW25Nxx_OBJ(ext_dev->obj)).page_num;
        ext_dev->page_size   = To_DevW25Nxx_API(ext_dev->api)->info(To_DevW25Nxx_OBJ(ext_dev->obj)).page_size;

        To_DevW25Nxx_OBJ(ext_dev->obj)->systick   = Storage_Dev_GetSstsTick;
        To_DevW25Nxx_OBJ(ext_dev->obj)->cs_ctl    = StoragePort_Api.cs_ctl;
        To_DevW25Nxx_OBJ(ext_dev->obj)->bus_tx    = StoragePort_Api.bus_tx;
        To_DevW25Nxx_OBJ(ext_dev->obj)->bus_rx    = StoragePort_Api.bus_rx;
        To_DevW25Nxx_OBJ(ext_dev->obj)->bus_trans = StoragePort_Api.bus_trans;
        To_DevW25Nxx_OBJ(ext_dev->obj)->delay_ms  = SrvOsCommon.delay_ms;

        return true;
    }
    
    return false;
}

static bool Storage_Dev_Init(StorageDevObj_TypeDef *ext_dev, uint16_t *p_type, uint16_t *p_code)
{
    uint8_t init_state = 0;

    if (ext_dev == NULL)
        return false;

    if (ext_dev->chip_type == Storage_ChipType_W25Qxx)
    {
        if ((To_DevW25Qxx_API(ext_dev->api)->init == NULL) || \
            (To_DevW25Qxx_API(ext_dev->api)->info == NULL))
            return false;

        init_state = To_DevW25Qxx_API(ext_dev->api)->init(To_DevW25Qxx_OBJ(ext_dev->obj));
        *p_type = To_DevW25Qxx_API(ext_dev->api)->info(To_DevW25Qxx_OBJ(ext_dev->obj)).prod_type;
        *p_code = To_DevW25Qxx_API(ext_dev->api)->info(To_DevW25Qxx_OBJ(ext_dev->obj)).prod_code;
    
        return ((DevW25Qxx_Error_List)init_state == DevW25Qxx_Ok) ? true : false;
    }
    else if (ext_dev->chip_type == Storage_ChipType_W25Nxx)
    {
        if ((To_DevW25Nxx_API(ext_dev->api)->init == NULL) || \
            (To_DevW25Nxx_API(ext_dev->api)->info == NULL))
            return false;

        init_state = To_DevW25Nxx_API(ext_dev->api)->init(To_DevW25Nxx_OBJ(ext_dev->obj));
        *p_type = To_DevW25Nxx_API(ext_dev->api)->info(To_DevW25Nxx_OBJ(ext_dev->obj)).prod_type;
        *p_code = To_DevW25Nxx_API(ext_dev->api)->info(To_DevW25Nxx_OBJ(ext_dev->obj)).prod_code;

        return ((DevW25Nxx_Error_List)init_state == DevW25Nxx_Ok) ? true : false;
    }

    return false;
}

static bool Storage_Dev_Write_Section(StorageDevObj_TypeDef *p_dev, uint32_t addr, uint8_t *p_data, uint16_t len)
{
    uint32_t write_cnt = 0;
    uint32_t addr_tmp = 0;

    if ((p_dev == NULL) || \
        (p_dev->api == NULL) || \
        (p_dev->obj == NULL) || \
        (p_data == NULL) || \
        (len == 0) || \
        ((addr % p_dev->sector_size) || \
        (len % p_dev->sector_size)))
        return false;

    write_cnt = len / p_dev->sector_size;
    addr_tmp = addr;

    for (uint8_t i = 0; i < write_cnt; i ++)
    {
        switch((uint8_t)p_dev->chip_type)
        {
            case Storage_ChipType_W25Qxx:
                /* erase sector */
                if (To_DevW25Qxx_API(p_dev->api)->erase_sector(To_DevW25Qxx_OBJ(p_dev->obj), addr_tmp) != DevW25Qxx_Ok)
                    return false;

                /* update sector */
                if (To_DevW25Qxx_API(p_dev->api)->write(To_DevW25Qxx_OBJ(p_dev->obj), addr_tmp, p_data, p_dev->sector_size))
                    return false;
                break;

            default: return false;
        }

        addr_tmp += p_dev->sector_size;
    }

    return true;
}

static bool Storage_Dev_Read_Section(StorageDevObj_TypeDef *p_dev, uint32_t addr, uint8_t *p_data, uint16_t len)
{
    uint32_t read_cnt = 0;
    uint32_t addr_tmp = 0;
    
    if ((p_dev == NULL) || \
        (p_dev->api == NULL) || \
        (p_dev->obj == NULL) || \
        (p_data == NULL) || \
        (len == 0) || \
        ((addr % p_dev->sector_size) || \
        (len % p_dev->sector_size)))
        return false;

    read_cnt = len / p_dev->sector_size;
    addr_tmp = addr;

    for (uint8_t i = 0; i < read_cnt; i++)
    {
        switch((uint8_t)p_dev->chip_type)
        {
            case Storage_ChipType_W25Qxx:
                /* read sector */
                if (To_DevW25Qxx_API(p_dev->api)->read(To_DevW25Qxx_OBJ(p_dev->obj), addr_tmp, p_data, len) != DevW25Qxx_Ok)
                {
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

static bool Storage_Dev_Erase_Section(StorageDevObj_TypeDef *p_dev, uint32_t addr, uint16_t len)
{
    uint32_t erase_cnt = 0;
    uint32_t addr_tmp = 0;

    if ((p_dev == NULL) || \
        (p_dev->api == NULL) || \
        (p_dev->obj == NULL) || \
        (len == 0) || \
        ((addr % p_dev->sector_size) || \
        (len % p_dev->sector_size)))
        return false;

    erase_cnt = len / p_dev->sector_size;
    addr_tmp = addr;

    for (uint8_t i = 0; i < erase_cnt; i ++)
    {
        switch((uint8_t)p_dev->chip_type)
        {
            case Storage_ChipType_W25Qxx:
                /* erase sector */
                if (To_DevW25Qxx_API(p_dev->api)->erase_sector(To_DevW25Qxx_OBJ(p_dev->obj), addr_tmp) != DevW25Qxx_Ok)
                    return false;
                break;

            default: return false;
        }

        addr_tmp += p_dev->sector_size;
    }

    return true;
}

