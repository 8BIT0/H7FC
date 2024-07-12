#include "Srv_BlackBox_Def.h"

#define COM_BLACKBOX_BAUDRATE 921600

/* external function */
static bool SrvCom_BlackBox_Init(SrvBlackBox_Log_Callback callback);
static bool SrvCom_BlackBox_PushData(uint8_t *p_data, uint16_t len);
static bool SrvCom_BlackBox_Read(uint32_t addr_offset, uint8_t *p_data, uint16_t len);
static bool SrvCom_BlackBox_Enable(void);
static bool SrvCom_BlackBox_Disable(void);

SrvBlackBox_TypeDef SrvCom_BlackBox = {
    .init = SrvCom_BlackBox_Init,
    .push = SrvCom_BlackBox_PushData,
    .read = SrvCom_BlackBox_Read,
    .enable = SrvCom_BlackBox_Enable,
    .disable = SrvCom_BlackBox_Disable,
};

static bool SrvCom_BlackBox_Init(SrvBlackBox_Log_Callback callback)
{
    return false;
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
