#include "Srv_BlackBox_Def.h"

/* external function */
static bool SrvCard_BlackBox_Init(SrvBlackBox_Log_Callback callback);
static bool SrvCard_BlackBox_PushData(uint8_t *p_data, uint16_t len);
static bool SrvCard_BlackBox_Read(uint32_t addr_offset, uint8_t *p_data, uint16_t len);
static bool SrvCard_BlackBox_Enable(void);
static bool SrvCard_BlackBox_Disable(void);

SrvBlackBox_TypeDef SrvCard_BlackBox = {
    .init = SrvCard_BlackBox_Init,
    .push = SrvCard_BlackBox_PushData,
    .read = SrvCard_BlackBox_Read,
    .enable = SrvCard_BlackBox_Enable,
    .disable = SrvCard_BlackBox_Disable,
};

static bool SrvCard_BlackBox_Init(SrvBlackBox_Log_Callback callback)
{
    return false;
}

static bool SrvCard_BlackBox_PushData(uint8_t *p_data, uint16_t len)
{
    return false;
}

static bool SrvCard_BlackBox_Read(uint32_t addr_offset, uint8_t *p_data, uint16_t len)
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

