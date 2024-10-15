#include "Storage_Dev_Port.h"
#include "Storage_Def.h"
#include "Srv_OsCommon.h"
#include "Dev_W25Qxx.h"
#include "Dev_W25Nxx.h"

#define Storage_Dev_GetSstsTick SrvOsCommon.get_os_ms
#define Storage_Dev_Malloc(x)   SrvOsCommon.malloc(x)
#define Storage_Dev_Free(x)     SrvOsCommon.free(x)

/* internal vriable */
static uint8_t read_tmp[Storage_TabSize * 2] __attribute__((aligned(4))) __attribute__((section(".Perph_Section"))) = {0};
static uint8_t write_tmp[Storage_TabSize * 2] __attribute__((aligned(4))) __attribute__((section(".Perph_Section"))) = {0};

/* external function */
static bool Storage_Dev_Set(StorageDevObj_TypeDef *ext_dev);
static bool Storage_Dev_Init(StorageDevObj_TypeDef *ext_dev, uint16_t *p_type, uint16_t *p_code);

static bool Storage_Dev_Write_Section(StorageDevObj_TypeDef *p_dev, uint32_t addr, uint8_t *p_data, uint16_t len);
static bool Storage_Dev_Read_Section(StorageDevObj_TypeDef *p_dev, uint32_t addr, uint8_t *p_data, uint16_t len);
static bool Storage_Dev_Erase_Section(StorageDevObj_TypeDef *p_dev, uint32_t addr, uint16_t len);

static bool Storage_Dev_Firmware_Format(StorageDevObj_TypeDef *p_dev, uint32_t addr, uint32_t firmware_size);
static bool Storage_Dev_Firmware_Read(StorageDevObj_TypeDef *p_dev, uint32_t base_addr, uint32_t addr_offset, uint8_t *p_data, uint16_t size);
static bool Storage_Dev_Firmware_Write(StorageDevObj_TypeDef *p_dev, uint32_t base_addr, uint32_t addr_offset, uint8_t *p_data, uint16_t size);

static bool Storage_Dev_Param_Read(StorageDevObj_TypeDef *p_dev, uint32_t base_addr, uint32_t addr_offset, uint8_t *p_data, uint32_t len);
static bool Storage_Dev_Param_Write(StorageDevObj_TypeDef *p_dev, uint32_t base_addr, uint32_t addr_offset, uint8_t *p_data, uint32_t len);
static bool Storage_Dev_Param_Erase(StorageDevObj_TypeDef *p_dev, uint32_t base_addr, uint32_t addr_offset, uint32_t len);

StorageDevApi_TypeDef StorageDev = {
    .set = Storage_Dev_Set,
    .init = Storage_Dev_Init,

    .write_phy_sec = Storage_Dev_Write_Section,
    .read_phy_sec = Storage_Dev_Read_Section,
    .erase_phy_sec = Storage_Dev_Erase_Section,

    .firmware_format = Storage_Dev_Firmware_Format,
    .firmware_read = Storage_Dev_Firmware_Read,
    .firmware_write = Storage_Dev_Firmware_Write,

    .param_read = Storage_Dev_Param_Read,
    .param_write = Storage_Dev_Param_Write,
    .param_erase = Storage_Dev_Param_Erase,
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
        if (ext_dev->obj == NULL)
            return false;

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
        if (ext_dev->obj == NULL)
            return false;
        
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
    
        ext_dev->start_addr  = W25QXX_BASE_ADDRESS;
        ext_dev->sector_num  = To_DevW25Qxx_API(ext_dev->api)->info(To_DevW25Qxx_OBJ(ext_dev->obj)).subsector_num;
        ext_dev->sector_size = To_DevW25Qxx_API(ext_dev->api)->info(To_DevW25Qxx_OBJ(ext_dev->obj)).subsector_size;
        ext_dev->total_size  = To_DevW25Qxx_API(ext_dev->api)->info(To_DevW25Qxx_OBJ(ext_dev->obj)).flash_size;
        ext_dev->page_num    = To_DevW25Qxx_API(ext_dev->api)->info(To_DevW25Qxx_OBJ(ext_dev->obj)).page_num;
        ext_dev->page_size   = To_DevW25Qxx_API(ext_dev->api)->info(To_DevW25Qxx_OBJ(ext_dev->obj)).page_size;

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

        ext_dev->start_addr  = W25NXX_BASE_ADDRESS;
        ext_dev->total_size  = To_DevW25Nxx_API(ext_dev->api)->info(To_DevW25Nxx_OBJ(ext_dev->obj)).flash_size;
        ext_dev->page_num    = To_DevW25Nxx_API(ext_dev->api)->info(To_DevW25Nxx_OBJ(ext_dev->obj)).page_num;
        ext_dev->page_size   = To_DevW25Nxx_API(ext_dev->api)->info(To_DevW25Nxx_OBJ(ext_dev->obj)).page_size;

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
                /* erase sector and update sector */
                if ((To_DevW25Qxx_API(p_dev->api)->erase_sector(To_DevW25Qxx_OBJ(p_dev->obj), addr_tmp) != DevW25Qxx_Ok) || \
                    (To_DevW25Qxx_API(p_dev->api)->write(To_DevW25Qxx_OBJ(p_dev->obj), addr_tmp, p_data, p_dev->sector_size) != DevW25Qxx_Ok))
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

static bool Storage_Dev_Firmware_Format(StorageDevObj_TypeDef *p_dev, uint32_t addr, uint32_t firmware_size)
{
    uint32_t format_size = firmware_size;
    uint32_t erase_addr = addr;

    if ((p_dev == NULL) || \
        ((format_size % Storage_TabSize) != 0) || \
        (p_dev->api == NULL) || \
        (p_dev->obj == NULL))
        return false;

    for (uint16_t i = 0; i < format_size / Storage_TabSize; i++)
    {
        switch (p_dev->chip_type)
        {
            case Storage_ChipType_W25Qxx:
                if (format_size == 0)
                    return true;

                if (To_DevW25Qxx_API(p_dev->api)->erase_sector(To_DevW25Qxx_OBJ(p_dev->obj), erase_addr) != DevW25Qxx_Ok)
                    return false;
            
                erase_addr += Storage_TabSize;
                format_size -= Storage_TabSize;
                break;

            default: break;
        }
    }

    return false;
}

static bool Storage_Dev_Firmware_Read(StorageDevObj_TypeDef *p_dev, uint32_t base_addr, uint32_t addr_offset, uint8_t *p_data, uint16_t size)
{
    uint32_t read_addr = 0;
    uint32_t section_addr = 0;
    uint32_t read_size = 0;

    if ((p_dev == NULL) || \
        (p_data == NULL) || \
        (size == 0) || \
        (sizeof(read_tmp) < size) || \
        (Storage_TabSize == 0))
        return false;
        
    read_addr = addr_offset + base_addr;
    while (true)
    {
        switch (p_dev->chip_type)
        {
            case Storage_ChipType_W25Qxx:
                section_addr = To_DevW25Qxx_API(p_dev->api)->get_section_start_addr(To_DevW25Qxx_OBJ(p_dev->obj), read_addr);
                if (To_DevW25Qxx_API(p_dev->api)->read(To_DevW25Qxx_OBJ(p_dev->obj), section_addr, read_tmp, Storage_TabSize) != DevW25Qxx_Ok)
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

                memcpy(p_data, &read_tmp[read_addr - section_addr], read_size);
                read_addr = section_addr + Storage_TabSize;

                if (size == 0)
                    return true;
            break;
        
            default: return false;
        }
    }
}

static bool Storage_Dev_Firmware_Write(StorageDevObj_TypeDef *p_dev, uint32_t base_addr, uint32_t addr_offset, uint8_t *p_data, uint16_t size)
{
    uint32_t write_addr = base_addr + addr_offset;
    uint32_t section_addr = 0;
    uint32_t write_size = 0;
    
    if ((p_dev == NULL) || \
        (Storage_TabSize == 0) || \
        (sizeof(read_tmp) < size) || \
        (p_dev->api == NULL) || \
        (p_dev->obj == NULL))
        return false;

    switch ((uint8_t)p_dev->chip_type)
    {
        case Storage_ChipType_W25Qxx:
            section_addr = To_DevW25Qxx_API(p_dev->api)->get_section_start_addr(To_DevW25Qxx_OBJ(p_dev->obj), write_addr);

            while (true)
            {
                if (size == 0)
                    return true;

                /* read section first */
                memset(read_tmp, 0, Storage_TabSize);
                if (To_DevW25Qxx_API(p_dev->api)->read(To_DevW25Qxx_OBJ(p_dev->obj), section_addr, read_tmp, Storage_TabSize) != DevW25Qxx_Ok)
                    return false;

                /* erase whole section */
                if (To_DevW25Qxx_API(p_dev->api)->erase_sector(To_DevW25Qxx_OBJ(p_dev->obj), section_addr) != DevW25Qxx_Ok)
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
                memcpy(&read_tmp[write_addr - section_addr], p_data, write_size);
                if (To_DevW25Qxx_API(p_dev->api)->write(To_DevW25Qxx_OBJ(p_dev->obj), section_addr, read_tmp, Storage_TabSize) != DevW25Qxx_Ok)
                    return false;

                /* update section address */
                p_data += write_size;
                write_addr += write_size;
                section_addr = To_DevW25Qxx_API(p_dev->api)->get_section_start_addr(To_DevW25Qxx_OBJ(p_dev->obj), write_addr);
            }
            break;

        default: return false;
    }
}

static bool Storage_Dev_Param_Read(StorageDevObj_TypeDef *p_dev, uint32_t base_addr, uint32_t addr_offset, uint8_t *p_data, uint32_t len)
{
    uint32_t read_start_addr = base_addr + addr_offset;
    uint32_t flash_end_addr = 0;
    uint32_t section_start_addr = 0;
    uint32_t next_read_addr = 0;
    uint32_t section_size = 0;
    uint32_t read_offset = 0;
    uint32_t read_len = len;

    if ((p_dev == NULL) || \
        (p_dev->api == NULL) || \
        (p_dev->obj == NULL) || \
        (sizeof(read_tmp) < len) || \
        (p_data == 0) || \
        (len == 0))
        return false;
        
    switch((uint8_t)p_dev->chip_type)
    {
        case Storage_ChipType_W25Qxx:
            section_size = To_DevW25Qxx_API(p_dev->api)->info(To_DevW25Qxx_OBJ(p_dev->obj)).subsector_size;
            /* get w25qxx device info */
            /* address check */
            flash_end_addr = To_DevW25Qxx_API(p_dev->api)->info(To_DevW25Qxx_OBJ(p_dev->obj)).start_addr;
            if (flash_end_addr > read_start_addr)
                return false;

            /* range check */
            flash_end_addr += To_DevW25Qxx_API(p_dev->api)->info(To_DevW25Qxx_OBJ(p_dev->obj)).flash_size;
            if ((len + read_start_addr) > flash_end_addr)
                return false;

            if (section_size == 0)
                return false;
                
            section_start_addr = To_DevW25Qxx_API(p_dev->api)->get_section_start_addr(To_DevW25Qxx_OBJ(p_dev->obj), read_start_addr);
            read_offset = read_start_addr - section_start_addr;
            if (section_size > sizeof(read_tmp))
                return false;

            while(true)
            {
                /* circumstances 1: store data size less than flash sector size and only none multiple sector read is needed */
                /* circumstances 2: store data size less than flash sector length but need to read from the end of the sector N to the start of the sector N + 1 */
                /* circumstances 3: store data size large than flash sector length */
                if (read_offset + read_len > section_size)
                    read_len = section_size - read_offset;

                /* read whole section */
                if (To_DevW25Qxx_API(p_dev->api)->read(To_DevW25Qxx_OBJ(p_dev->obj), section_start_addr, read_tmp, section_size) != DevW25Qxx_Ok)
                    return false;
            
                memcpy(p_data, read_tmp + read_offset, read_len);
                memset(read_tmp, 0, section_size);

                len -= read_len;
                if (len == 0)
                    return true;
            
                read_offset = 0;
                next_read_addr = To_DevW25Qxx_API(p_dev->api)->get_section_start_addr(To_DevW25Qxx_OBJ(p_dev->obj), section_start_addr + read_len);
                if (next_read_addr == section_start_addr)
                    read_offset = read_len;
                
                p_data += read_len;
                read_len = len;
                section_start_addr = next_read_addr;
            }
            break;

        default: return false;
    }

    return false;
}

static bool Storage_Dev_Param_Write(StorageDevObj_TypeDef *p_dev, uint32_t base_addr, uint32_t addr_offset, uint8_t *p_data, uint32_t len)
{
    uint32_t write_start_addr = base_addr + addr_offset;
    uint32_t flash_end_addr = 0;
    uint32_t section_start_addr = 0;
    uint32_t next_write_addr = 0;
    uint32_t section_size = 0;
    uint32_t write_offset = 0;
    uint32_t write_len = len;

    if ((p_dev == NULL) || \
        (p_dev->api == NULL) || \
        (p_dev->obj == NULL) || \
        (p_data == NULL) || \
        (len == 0))
        return false;

    switch((uint8_t)p_dev->chip_type)
    {
        case Storage_ChipType_W25Qxx:
            section_size = To_DevW25Qxx_API(p_dev->api)->info(To_DevW25Qxx_OBJ(p_dev->obj)).subsector_size;
            /* get w25qxx device info */
            /* address check */
            flash_end_addr = To_DevW25Qxx_API(p_dev->api)->info(To_DevW25Qxx_OBJ(p_dev->obj)).start_addr;
            if (flash_end_addr > write_start_addr)
                return false;

            /* range check */
            flash_end_addr += To_DevW25Qxx_API(p_dev->api)->info(To_DevW25Qxx_OBJ(p_dev->obj)).flash_size;
            if ((len + write_start_addr) > flash_end_addr)
                return false;
            
            if (section_size == 0)
                return false;
                
            section_start_addr = To_DevW25Qxx_API(p_dev->api)->get_section_start_addr(To_DevW25Qxx_OBJ(p_dev->obj), write_start_addr);
            write_offset = write_start_addr - section_start_addr;
            if (section_size > sizeof(write_tmp))
                return false;

            while(true)
            {
                /* circumstances 1: store data size less than flash sector size and only none multiple sector write is needed */
                /* circumstances 2: store data size less than flash sector length but need to write from the end of the sector N to the start of the sector N + 1 */
                /* circumstances 3: store data size large than flash sector length */
                /* read whole section */
                if (To_DevW25Qxx_API(p_dev->api)->read(To_DevW25Qxx_OBJ(p_dev->obj), section_start_addr, write_tmp, section_size) != DevW25Qxx_Ok)
                    return false;

                /* erase whole section */
                if (To_DevW25Qxx_API(p_dev->api)->erase_sector(To_DevW25Qxx_OBJ(p_dev->obj), section_start_addr) != DevW25Qxx_Ok)
                    return false;

                /* update whole section */
                if (write_offset + write_len > section_size)
                    write_len = section_size - write_offset;
                
                /* copy data to section data read out */
                memcpy(write_tmp + write_offset, p_data, write_len);

                DevW25Qxx_Error_List state = To_DevW25Qxx_API(p_dev->api)->write(To_DevW25Qxx_OBJ(p_dev->obj), section_start_addr, write_tmp, section_size);

                /* clear cache buff */
                memset(write_tmp, 0, section_size);
                
                len -= write_len;
                if (state == DevW25Qxx_Ok)
                {
                    if (len == 0)
                        return true;
                }
                else
                    return false;

                write_offset = 0;
                next_write_addr = To_DevW25Qxx_API(p_dev->api)->get_section_start_addr(To_DevW25Qxx_OBJ(p_dev->obj), section_start_addr + write_len);
                if (next_write_addr == section_start_addr)
                    write_offset = write_len;

                p_data += write_len;
                write_len = len;
                section_start_addr = next_write_addr; 
            }
            return false;

        default: return false;
    }

    return false;
}

static bool Storage_Dev_Param_Erase(StorageDevObj_TypeDef *p_dev, uint32_t base_addr, uint32_t addr_offset, uint32_t len)
{
    uint32_t erase_start_addr = base_addr + addr_offset;

    if ((p_dev == NULL) || \
        (p_dev->api == NULL) || \
        (p_dev->obj == NULL) || \
        (len == 0))
        return false;

    /* erase external flash sector */
    switch((uint8_t)p_dev->chip_type)
    {
        case Storage_ChipType_W25Qxx:
                /* get w25qxx device info */
                /* address check */
                if (erase_start_addr < To_DevW25Qxx_API(p_dev->api)->info(To_DevW25Qxx_OBJ(p_dev->obj)).start_addr)
                    return false;

                /* W25Qxx device read */
                if (To_DevW25Qxx_API(p_dev->api)->erase_sector(To_DevW25Qxx_OBJ(p_dev->obj), erase_start_addr) == DevW25Qxx_Ok)
                    return true;
            break;

        default: return false;
    }
    
    return false;
}
