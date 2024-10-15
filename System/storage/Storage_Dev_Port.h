#ifndef __STORAGE_DEV_PORT_H
#define __STORAGE_DEV_PORT_H

#include <stdint.h>
#include <string.h>
#include <stdint.h>
#include "Storage_Bus_Port.h"

typedef enum
{
    Storage_Chip_None = 0,
    Storage_ChipType_W25Qxx,
    Storage_ChipType_W25Nxx,
    Storage_ChipType_All,
} Storage_ExtFlashChipType_List;

/* hadware flash chip info */
typedef struct
{
    Storage_ExtFlash_BusType_List bus_type;
    Storage_ExtFlashChipType_List chip_type;

    uint32_t start_addr;
    uint32_t total_size;

    uint32_t page_num;
    uint32_t page_size;

    uint32_t bank_num;
    uint32_t bank_size;

    uint32_t block_num;
    uint32_t block_size;

    uint32_t sector_num;
    uint32_t sector_size;

    void *obj;
    void *api;
} StorageDevObj_TypeDef;

typedef struct
{
    bool (*set)(StorageDevObj_TypeDef *ext_dev);
    bool (*init)();
} StorageDevApi_TypeDef;

extern StorageDevApi_TypeDef StorageDev;

#endif
