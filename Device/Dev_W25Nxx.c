#include "Dev_W25Nxx.h"

/* test code */
#include "HW_Def.h"
#include "debug_util.h"

#define W25NXX_TAG "[ W25NXX INFO ] "
#define W25NXX_INFO(fmt, ...) Debug_Print(&DebugPort, W25NXX_TAG, fmt, ##__VA_ARGS__)
/* test code */

#define W25NXX_BUS_COMMU_TIMEOUT 100 /* unit: ms */

/* internal function */
static bool DevW25Nxx_Write(DevW25NxxObj_TypeDef *dev, uint8_t *p_tx, uint16_t len);
static bool DevW25Nxx_Read(DevW25NxxObj_TypeDef *dev, uint8_t *p_rx, uint16_t len);
static bool DevW25Nxx_Trans(DevW25NxxObj_TypeDef *dev, uint8_t *p_tx, uint8_t *p_rx, uint16_t len);
static DevW25Nxx_ProdType_List DevW25Nxx_Get_ProductID(DevW25NxxObj_TypeDef *dev);
static bool DevW25Nxx_Soft_Reset(DevW25NxxObj_TypeDef *dev);
static DevW25Nxx_Error_List DevW25Nxx_Check_Read_Status(DevW25NxxObj_TypeDef *dev);

/* external function */
static DevW25Nxx_Error_List DevW25Nxx_Init(DevW25NxxObj_TypeDef *dev);
static DevW25Nxx_DeviceInfo_TypeDef DevW25Nxx_Get_Info(DevW25NxxObj_TypeDef *dev);
static uint32_t DevW25Nxx_Get_Page(DevW25NxxObj_TypeDef *dev, uint32_t addr);

DevW25Nxx_TypeDef DevW25Nxx = {
    .init = DevW25Nxx_Init,
    .info = DevW25Nxx_Get_Info,
    .get_page = DevW25Nxx_Get_Page,
};

static bool DevW25Nxx_Write(DevW25NxxObj_TypeDef *dev, uint8_t *p_tx, uint16_t len)
{
    uint16_t tx_out = 0;

    if ((dev == NULL) || \
        (dev->bus_tx == NULL) || \
        (p_tx == NULL) || \
        (len == 0))
        return false;

    tx_out = dev->bus_tx(p_tx, len, W25NXX_BUS_COMMU_TIMEOUT);

    return tx_out ? true : false;
}

static bool DevW25Nxx_Read(DevW25NxxObj_TypeDef *dev, uint8_t *p_rx, uint16_t len)
{
    uint16_t rx_out = 0;

    if ((dev == NULL) || \
        (dev->bus_rx == NULL) || \
        (p_rx == NULL) || \
        (len == 0))
        return false;

    rx_out = dev->bus_rx(p_rx, len, W25NXX_BUS_COMMU_TIMEOUT);

    return rx_out ? true : false;
}

static bool DevW25Nxx_Trans(DevW25NxxObj_TypeDef *dev, uint8_t *p_tx, uint8_t *p_rx, uint16_t len)
{
    uint16_t trans_out = 0;

    if ((dev == NULL) || \
        (dev->bus_trans == NULL) || \
        (p_rx == NULL) || \
        (p_tx == NULL) || \
        (len == 0))
        return false;

    trans_out = dev->bus_trans(p_tx, p_rx, len, W25NXX_BUS_COMMU_TIMEOUT);

    return trans_out ? true : false;
}

static DevW25Nxx_Error_List DevW25Nxx_Init(DevW25NxxObj_TypeDef *dev)
{
    DevW25Nxx_Error_List err = DevW25Nxx_Ok;
    uint32_t sys_time = 0;

    if ((dev == NULL) || \
        (dev->delay_ms == NULL) || \
        (dev->systick == NULL))
        return DevW25Nxx_Error;

    dev->init_state = false;

    /* check read status */
    err = DevW25Nxx_Check_Read_Status(dev);
    if (err == DevW25Nxx_Error)
        return DevW25Nxx_Error;
    
    sys_time = dev->systick();
    while (err == DevW25Nxx_Busy)
    {
        if ((dev->systick() - sys_time) >= W25NXX_BUS_COMMU_TIMEOUT)
        {
            W25NXX_INFO("time out\r\n");
            return DevW25Nxx_TimeOut;
        }

        dev->delay_ms(1);
        err = DevW25Nxx_Check_Read_Status(dev);
        if (err == DevW25Nxx_Error)
            return DevW25Nxx_Error;
    }

    /* get product id */
    dev->prod_type = DevW25Nxx_Get_ProductID(dev);
    if (dev->prod_type == DevW25N_None)
        return DevW25Nxx_Error;

    /* soft reset */
    if (!DevW25Nxx_Soft_Reset(dev))
        return DevW25Nxx_Error;

    dev->delay_ms(100);
    dev->init_state = true;

    return DevW25Nxx_Ok;
}

static bool DevW25Nxx_Soft_Reset(DevW25NxxObj_TypeDef *dev)
{
    bool state = false;
    uint8_t tx_tmp[2] = {0};

    if ((dev == NULL) || \
        (dev->cs_ctl == NULL))
        return false;

    memset(tx_tmp, 0, sizeof(tx_tmp));
    tx_tmp[0] = W25NXX_RESET_CMD;

    dev->cs_ctl(false);
    state = DevW25Nxx_Write(dev, tx_tmp, sizeof(tx_tmp));
    dev->cs_ctl(true);

    return state;
}

static DevW25Nxx_ProdType_List DevW25Nxx_Get_ProductID(DevW25NxxObj_TypeDef *dev)
{
    uint8_t tx_tmp[2] = {0};
    uint8_t rx_tmp[3] = {0};
    uint32_t ID = 0;
    bool state = false;

    if ((dev == NULL) || \
        (dev->cs_ctl == NULL))
        return DevW25N_None;

    memset(tx_tmp, 0, sizeof(tx_tmp));
    memset(rx_tmp, 0, sizeof(rx_tmp));
    tx_tmp[0] = W25NXX_JEDEC_ID;

    dev->cs_ctl(false);
    state = DevW25Nxx_Write(dev, tx_tmp, sizeof(tx_tmp));
    state &= DevW25Nxx_Read(dev, rx_tmp, sizeof(rx_tmp));
    dev->cs_ctl(true);

    if (!state)
        return DevW25N_None;

    ((uint8_t *)&ID)[3] = 0;
    ((uint8_t *)&ID)[2] = rx_tmp[0];
    ((uint8_t *)&ID)[1] = rx_tmp[1];
    ((uint8_t *)&ID)[0] = rx_tmp[2];
    
    dev->prod_code = ID;
    switch (ID)
    {
        case W25N01GVZEIG_ID: W25NXX_INFO("type W25N01G\r\n"); return DevW25N_01;
        default: W25NXX_INFO("type Unknown 0x%08x\r\n", ID); return DevW25N_None;
    }

    return DevW25N_None;
}

static DevW25Nxx_Error_List DevW25Nxx_Check_Read_Status(DevW25NxxObj_TypeDef *dev)
{
    bool state = false;
    uint8_t cmd[3] = {W25NXX_READ_STATUS_CMD, W25NXX_SR2_ADDR, 0};
    uint8_t reg_val[3] = {0, 0, 0};
    DevW25Nxx_SR2_TypeDef sr2;

    if ((dev == NULL) || \
        (dev->cs_ctl == NULL))
        return DevW25Nxx_Error;

    dev->cs_ctl(false);
    state = DevW25Nxx_Trans(dev, cmd, reg_val, sizeof(cmd));
    dev->cs_ctl(true);
    if (!state)
        return DevW25Nxx_Error;

    sr2.val = reg_val[2];
    if (sr2.bit.BUSY == 0)
        return DevW25Nxx_Ok;

    return DevW25Nxx_Busy;
}

static DevW25Nxx_DeviceInfo_TypeDef DevW25Nxx_Get_Info(DevW25NxxObj_TypeDef *dev)
{
    DevW25Nxx_DeviceInfo_TypeDef info;
    memset(&info, 0, sizeof(DevW25Nxx_DeviceInfo_TypeDef));

    switch ((uint8_t) dev->prod_type)
    {
        case DevW25N_01:
            info.flash_size = W25N01GV_FLASH_SIZE;
            info.page_num = W25N01GV_PAGE_NUM;
            info.page_size = W25N01GV_PAGE_SIZE + W25N0GV_ECC_INFO_SIZE;
            info.block_num = W25N01GV_BLOCK_NUM;
            info.block_size = W25N01GV_BLOCK_SIZE;
            info.prod_code = dev->prod_code;
            info.prod_type = dev->prod_type;
            info.start_addr = W25NXX_BASE_ADDRESS;
            break;
    
        default: break;
    }

    return info;
}

static uint32_t DevW25Nxx_Get_Page(DevW25NxxObj_TypeDef *dev, uint32_t addr)
{
    if ((dev == NULL) || \
        !dev->init_state || \
        (dev->prod_type == DevW25N_None) || \
        (dev->prod_type > DevW25N_02) || \
        (addr > W25N01GV_FLASH_SIZE))
        return 0;

    return (addr / W25N01GV_PAGE_SIZE);
}
