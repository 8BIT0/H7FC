#ifndef __SRV_BLACKBOX_DEF_H
#define __SRV_BLACKBOX_DEF_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "Srv_OsCommon.h"
#include "HW_Def.h"

typedef void (*SrvBlackBox_Log_Callback)(void);
typedef bool (*SrvBlackBox_write_callback)(uint32_t addr, uint8_t *p_buff, uint16_t len);
typedef bool (*SrvBlackBox_read_callback)(uint32_t addr, uint8_t *p_buff, uint16_t len);

typedef struct
{
    uint32_t phy_start_addr;    /* physical device start address */
    uint32_t str_start_addr;    /* storage section start address */

    uint32_t total_str_size;
    uint32_t sector_size;
} SrvBlackBox_DevInfo_TypeDef;

typedef struct
{
    uint32_t (*init)(SrvBlackBox_Log_Callback callback, SrvBlackBox_DevInfo_TypeDef devinfo);
    bool (*enable)(void);
    bool (*disable)(SrvBlackBox_write_callback p_write);
    bool (*push)(SrvBlackBox_write_callback p_write, uint8_t *p_data, uint32_t len);
    bool (*read)(SrvBlackBox_read_callback p_read, uint32_t addr_offset, uint8_t *p_data, uint32_t len);
    bool (*get_info)(SrvBlackBox_read_callback p_read, uint32_t *cnt, uint32_t *size, bool *enable_state);
} SrvBlackBox_TypeDef;

extern SrvBlackBox_TypeDef SrvPort_BlackBox;
extern SrvBlackBox_TypeDef SrvCard_BlackBox;
extern SrvBlackBox_TypeDef SrvChip_BlackBox;

#endif
