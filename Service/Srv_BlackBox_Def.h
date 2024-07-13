#ifndef __SRV_BLACKBOX_DEF_H
#define __SRV_BLACKBOX_DEF_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "Srv_OsCommon.h"
#include "HW_Def.h"

typedef void (*SrvBlackBox_Log_Callback)(void);

typedef struct
{
    uint32_t (*init)(SrvBlackBox_Log_Callback callback);
    bool (*enable)(void);
    bool (*disable)(void);
    bool (*push)(uint8_t *p_data, uint16_t len);
    bool (*read)(uint32_t addr_offset, uint8_t *p_data, uint16_t len);
    bool (*get_info)(uint32_t *cnt, uint32_t *size, bool *enable_state);
} SrvBlackBox_TypeDef;

extern SrvBlackBox_TypeDef SrvCom_BlackBox;
extern SrvBlackBox_TypeDef SrvCard_BlackBox;
extern SrvBlackBox_TypeDef SrvChip_BlackBox;

#endif
