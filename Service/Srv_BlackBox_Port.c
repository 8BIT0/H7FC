#include "Srv_BlackBox_Def.h"

#define Port_BLACKBOX_BAUDRATE 921600

/* external function */
static uint32_t SrvPort_BlackBox_Init(SrvBlackBox_Log_Callback callback, SrvBlackBox_DevInfo_TypeDef devinfo);
static bool SrvPort_BlackBox_PushData(SrvBlackBox_write_callback p_write, uint8_t *p_data, uint16_t len);
static bool SrvPort_BlackBox_Read(SrvBlackBox_read_callback p_read, uint32_t addr_offset, uint8_t *p_data, uint16_t len);
static bool SrvPort_BlackBox_GetInfo(SrvBlackBox_read_callback p_read, uint32_t *cnt, uint32_t *size, bool *enable_state);
static bool SrvPort_BlackBox_Enable(void);
static bool SrvPort_BlackBox_Disable(SrvBlackBox_write_callback p_write);

SrvBlackBox_TypeDef SrvPort_BlackBox = {
    .init = SrvPort_BlackBox_Init,
    .push = SrvPort_BlackBox_PushData,
    .read = SrvPort_BlackBox_Read,
    .enable = SrvPort_BlackBox_Enable,
    .disable = SrvPort_BlackBox_Disable,
    .get_info = SrvPort_BlackBox_GetInfo,
};

static uint32_t SrvPort_BlackBox_Init(SrvBlackBox_Log_Callback callback, SrvBlackBox_DevInfo_TypeDef devinfo)
{
    return 0;
}

static bool SrvPort_BlackBox_PushData(SrvBlackBox_write_callback p_write, uint8_t *p_data, uint16_t len)
{
    return false;
}

static bool SrvPort_BlackBox_Read(SrvBlackBox_read_callback p_read, uint32_t addr_offset, uint8_t *p_data, uint16_t len)
{
    return false;
}

static bool SrvPort_BlackBox_Enable(void)
{
    return false;
}

static bool SrvPort_BlackBox_Disable(SrvBlackBox_write_callback p_write)
{
    return false;
}

static bool SrvPort_BlackBox_GetInfo(SrvBlackBox_read_callback p_read, uint32_t *cnt, uint32_t *size, bool *enable_state)
{
    if (cnt && size && enable_state)
    {
        *cnt = 0;
        *size = 0;
        *enable_state = 0;
    }

    return false;
}
