#include "Dev_W25Qxx.h"

static DevW25Qxx_Error_List DevW25Qxx_Read(DevW25QxxObj_TypeDef dev, uint32_t addr, uint32_t *rx, uint32_t size)
{
    return DevW25Qxx_Ok;
}

static DevW25Qxx_Error_List DevW25Qxx_Write(DevW25QxxObj_TypeDef dev, uint32_t addr, uint8_t *tx, uint32_t size)
{
    return DevW25Qxx_Ok;
}

static DevW25Qxx_Error_List DevW25Qxx_Erase(DevW25QxxObj_TypeDef dev, DevW25Qxx_EraseType_List type, uint32_t addr, uint32_t size)
{
    return DevW25Qxx_Ok;
}

static bool DevW25Qxx_ReadID(DevW25QxxObj_TypeDef *dev)
{
    if ((dev == NULL) || (dev->cs_ctl == NULL))
        return false;

    dev->cs_ctl(true);

    dev->cs_ctl(false);

    return true;
}

static bool DevW25Qxx_Init(DevW25QxxObj_TypeDef *dev)
{
    if ((dev == NULL) ||
        (dev->cs_init == NULL) ||
        (dev->cs_ctl == NULL) ||
        (dev->bus_init == NULL) ||
        (dev->bus_trans_buff == NULL))
        return false;

    dev->bus_init();
    dev->cs_init();

    return true;
}
