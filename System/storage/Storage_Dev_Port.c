#include "Storage_Dev_Port.h"
#include "Srv_OsCommon.h"
#include "Dev_W25Qxx.h"
#include "Dev_W25Nxx.h"

#define Storage_Dev_GetSstsTick SrvOsCommon.get_os_ms
#define Storage_Dev_Malloc(x)   SrvOsCommon.malloc(x)
#define Storage_Dev_Free(x)     SrvOsCommon.free(x)

/* external function */
static bool Storage_Dev_Set(StorageDevObj_TypeDef *ext_dev);

StorageDevApi_TypeDef StorageDev = {
    .set = Storage_Dev_Set,
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
