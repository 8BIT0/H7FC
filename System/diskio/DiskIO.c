#include "DiskIO.h"
#include "Dev_W25Qxx.h"
#include "IO_Definition.h"

#if (EXTERNAL_STORAGE_MODULE == 1)
DevW25QxxObj_TypeDef W25Q64_Obj = {
    .bus_type = DevW25Qxx_Norm_SpiBus,
    .BusPort = SPI1,
};

DevW25QxxPin_Config_TypeDef W25Qxx_Pin = {
    // .pin_Alternate =,

    // .pin_clk =,
    // .pin_miso =,
    // .pin_mosi =,

    // .port_clk =,
    // .port_miso =,
    // .port_mosi =,
};

bool ExtDisk_Init(void)
{
    bool init_state = false;

    // DevW25Q64.init(W25Q64_Obj);

    return init_state;
}
#endif

bool IntDisk_Init(void)
{
    bool init_state = false;

    return init_state;
}

bool Disk_Init(void)
{
#if (EXTERNAL_STORAGE_MODULE == 1)
    if (!ExtDisk_Init())
        return false;
#endif

    if (!IntDisk_Init())
        return false;

    return true;
}
