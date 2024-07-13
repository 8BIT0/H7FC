#include "Srv_BlackBox_Def.h"

#define COM_BLACKBOX_BAUDRATE 921600

/* external function */
static uint32_t SrvCom_BlackBox_Init(SrvBlackBox_Log_Callback callback);
static bool SrvCom_BlackBox_PushData(uint8_t *p_data, uint16_t len);
static bool SrvCom_BlackBox_Read(uint32_t addr_offset, uint8_t *p_data, uint16_t len);
static bool SrvCom_BlackBox_GetInfo(uint32_t *cnt, uint32_t *size, bool *enable_state);
static bool SrvCom_BlackBox_Enable(void);
static bool SrvCom_BlackBox_Disable(void);

SrvBlackBox_TypeDef SrvCom_BlackBox = {
    .init = SrvCom_BlackBox_Init,
    .push = SrvCom_BlackBox_PushData,
    .read = SrvCom_BlackBox_Read,
    .enable = SrvCom_BlackBox_Enable,
    .disable = SrvCom_BlackBox_Disable,
    .get_info = SrvCom_BlackBox_GetInfo,
};

static uint32_t SrvCom_BlackBox_Init(SrvBlackBox_Log_Callback callback)
{
    return 0;
}

static bool SrvCom_BlackBox_PushData(uint8_t *p_data, uint16_t len)
{
    return false;
}

static bool SrvCom_BlackBox_Read(uint32_t addr_offset, uint8_t *p_data, uint16_t len)
{
    return false;
}

static bool SrvCom_BlackBox_Enable(void)
{
    return false;
}

static bool SrvCom_BlackBox_Disable(void)
{
    return false;
}

static bool SrvCom_BlackBox_GetInfo(uint32_t *cnt, uint32_t *size, bool *enable_state)
{
    if (cnt && size && enable_state)
    {
        *cnt = 0;
        *size = 0;
        *enable_state = 0;
    }

    return false;
}
