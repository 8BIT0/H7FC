#include "Srv_BlackBox_Def.h"

/* external function */
static uint32_t SrvCard_BlackBox_Init(SrvBlackBox_Log_Callback callback);
static bool SrvCard_BlackBox_PushData(uint8_t *p_data, uint32_t len);
static bool SrvCard_BlackBox_Read(uint32_t addr_offset, uint8_t *p_data, uint32_t len);
static bool SrvCard_BlackBox_GetInfo(uint32_t *cnt, uint32_t *size, bool *enable_state);
static bool SrvCard_BlackBox_Enable(void);
static bool SrvCard_BlackBox_Disable(void);

SrvBlackBox_TypeDef SrvCard_BlackBox = {
    .init = SrvCard_BlackBox_Init,
    .push = SrvCard_BlackBox_PushData,
    .read = SrvCard_BlackBox_Read,
    .enable = SrvCard_BlackBox_Enable,
    .disable = SrvCard_BlackBox_Disable,
    .get_info = SrvCard_BlackBox_GetInfo,
};

static uint32_t SrvCard_BlackBox_Init(SrvBlackBox_Log_Callback callback)
{
    return 0;
}

static bool SrvCard_BlackBox_PushData(uint8_t *p_data, uint32_t len)
{
    return false;
}

static bool SrvCard_BlackBox_Read(uint32_t addr_offset, uint8_t *p_data, uint32_t len)
{
    return false;
}

static bool SrvCard_BlackBox_Enable(void)
{
    return false;
}

static bool SrvCard_BlackBox_Disable(void)
{
    return false;
}

static bool SrvCard_BlackBox_GetInfo(uint32_t *cnt, uint32_t *size, bool *enable_state)
{
    if (cnt && size && enable_state)
    {
        *cnt = 0;
        *size = 0;
        *enable_state = 0;
    }

    return false;
}

