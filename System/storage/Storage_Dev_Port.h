#ifndef __STORAGE_DEV_PORT_H
#define __STORAGE_DEV_PORT_H

#include <stdint.h>
#include <string.h>
#include <stdint.h>
#include "Storage_Bus_Port.h"

#define To_StorageDevObj_Ptr(x) ((StorageDevObj_TypeDef *)x)

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
    bool (*init)(StorageDevObj_TypeDef *ext_dev, uint16_t *p_type, uint16_t *p_code);

    bool (*write_phy_sec)(StorageDevObj_TypeDef *p_dev, uint32_t addr, uint8_t *p_data, uint16_t len);
    bool (*read_phy_sec)(StorageDevObj_TypeDef *p_dev, uint32_t addr, uint8_t *p_data, uint16_t len);
    bool (*erase_phy_sec)(StorageDevObj_TypeDef *p_dev, uint32_t addr, uint16_t len);

    bool (*firmware_format)(StorageDevObj_TypeDef *p_dev, uint32_t addr, uint32_t firmware_size);
    bool (*firmware_read)(StorageDevObj_TypeDef *p_dev, uint32_t base_addr, uint32_t addr_offset, uint8_t *p_data, uint16_t size);
    bool (*firmware_write)(StorageDevObj_TypeDef *p_dev, uint32_t base_addr, uint32_t addr_offset, uint8_t *p_data, uint16_t size);

    bool (*param_write)(StorageDevObj_TypeDef *p_dev, uint32_t base_addr, uint32_t addr_offset, uint8_t *p_data, uint32_t len);
    bool (*param_read)(StorageDevObj_TypeDef *p_dev, uint32_t base_addr, uint32_t addr_offset, uint8_t *p_data, uint32_t len);
    bool (*param_erase)(StorageDevObj_TypeDef *p_dev, uint32_t base_addr, uint32_t addr_offset, uint32_t len);
} StorageDevApi_TypeDef;

extern StorageDevApi_TypeDef StorageDev;

#endif
