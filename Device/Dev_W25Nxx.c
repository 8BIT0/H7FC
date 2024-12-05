/*
 * Author: 8_B!T0
 * Still in developping
 */
#include "Dev_W25Nxx.h"

/* test code */
#include "HW_Def.h"
#include "debug_util.h"

#define W25NXX_TAG "[ W25NXX INFO ] "
#define W25NXX_INFO(fmt, ...) Debug_Print(&DebugPort, W25NXX_TAG, fmt, ##__VA_ARGS__)
/* test code */

#define W25NXX_SET_SINGLE_BIT(x)    (x = true)
#define W25NXX_EXTENSION_DATA_INDEX 2048
#define W25NXX_BUS_COMMU_TIMEOUT    100 /* unit: ms */
#define ConvertToSR0_RegFormat(x)   ((DevW25Nxx_SR0_TypeDef *)x)
#define ConvertToSR1_RegFormat(x)   ((DevW25Nxx_SR1_TypeDef *)x)
#define ConvertToSR2_RegFormat(x)   ((DevW25Nxx_SR2_TypeDef *)x)

typedef struct
{
    uint16_t p_addr;    /* physical address */
    uint16_t l_addr;    /* logic address */
} W25Nxx_BBLUT_TypeDef;

typedef struct
{
    uint8_t addr_h;
    uint8_t addr_l;
} W25Nxx_FormatAddr_TypeDef;

typedef union
{
    uint16_t val;
    struct
    {
        uint16_t b_index : 10;
        uint16_t p_index : 6;
    } bit;
} DevW25Nx_PageAddr_TypeDef;

/* internal function */
static bool DevW25Nxx_Trans_Buff(DevW25NxxObj_TypeDef *dev, uint8_t *p_tx, uint16_t tx_len, uint8_t *p_buf, uint32_t size);
static bool DevW25Nxx_Trans_Duplex(DevW25NxxObj_TypeDef *dev, uint8_t *p_tx, uint8_t *p_rx, uint16_t len);
static bool DevW25Nxx_Trans_Recevice(DevW25NxxObj_TypeDef *dev, uint8_t *p_tx, uint16_t tx_len, uint8_t *p_rx, uint16_t rx_len);
static DevW25Nxx_ProdType_List DevW25Nxx_Get_ProductID(DevW25NxxObj_TypeDef *dev);
static bool DevW25Nxx_Soft_Reset(DevW25NxxObj_TypeDef *dev);
static DevW25Nxx_Error_List DevW25Nxx_Check_Reg_Status(DevW25NxxObj_TypeDef *dev, uint8_t reg_addr, uint8_t *reg);
static DevW25Nxx_Error_List DevW25Nxx_WriteEn(DevW25NxxObj_TypeDef *dev, bool en);
static DevW25Nxx_Error_List DevW25Nxx_WriteReg_Set(DevW25NxxObj_TypeDef *dev, uint8_t reg_addr, uint8_t field);
static bool DevW25Nxx_Wait(DevW25NxxObj_TypeDef *dev, DevW25Nxx_SR2_BitField_TypeDef bit);
static DevW25Nxx_Error_List DevW25Nxx_Send_CMD(DevW25NxxObj_TypeDef *dev, uint8_t reg, uint32_t page_addr);
static uint32_t DevW25Nxx_Get_Column(uint32_t addr);
static uint32_t DevW25Nxx_Get_Block(uint32_t addr);
static W25Nxx_FormatAddr_TypeDef DevW25Nxx_FormatAddr(uint32_t addr);
static DevW25Nxx_Error_List DevW25Nxx_Read_ExtensionOnBlock(DevW25NxxObj_TypeDef *dev, uint32_t addr, uint8_t *p_data, uint32_t size);
static DevW25Nxx_Error_List DevW25Nxx_CheckBlock(DevW25NxxObj_TypeDef *dev, uint32_t block_index);
static DevW25Nxx_Error_List DevW25Nxx_ReadDataBuffer(DevW25NxxObj_TypeDef *dev, uint32_t colum_addr, uint8_t *p_buf, uint32_t size);
static bool DevW25Nxx_Check_PageAddr(DevW25NxxObj_TypeDef *dev, uint32_t page_addr);
static DevW25Nxx_Error_List DevW25Nxx_LoadData(DevW25NxxObj_TypeDef *dev, uint32_t column_addr, uint8_t *p_buf, uint16_t len);

/* external function */
static DevW25Nxx_Error_List DevW25Nxx_Init(DevW25NxxObj_TypeDef *dev);
static DevW25Nxx_DeviceInfo_TypeDef DevW25Nxx_Get_Info(DevW25NxxObj_TypeDef *dev);
static uint32_t DevW25Nxx_Get_Page(DevW25NxxObj_TypeDef *dev, uint32_t addr);
static DevW25Nxx_Error_List DevW25Nxx_Read_Page(DevW25NxxObj_TypeDef *dev, uint32_t addr, uint8_t *p_data, uint32_t size);
static DevW25Nxx_Error_List DevW25Nxx_Write_Page(DevW25NxxObj_TypeDef *dev, uint32_t addr, uint8_t *p_data, uint32_t size);
static DevW25Nxx_Error_List DevW25Nxx_Erase_PageOnBlock(DevW25NxxObj_TypeDef *dev, uint32_t addr);

DevW25Nxx_TypeDef DevW25Nxx = {
    .init = DevW25Nxx_Init,
    .info = DevW25Nxx_Get_Info,
    .get_page = DevW25Nxx_Get_Page,
    .read = DevW25Nxx_Read_Page,
};

static bool DevW25Nxx_Trans_Buff(DevW25NxxObj_TypeDef *dev, uint8_t *p_tx, uint16_t tx_len, uint8_t *p_buf, uint32_t size)
{
    bool state = false;

    if ((dev == NULL) || \
        (dev->bus_tx == NULL) || \
        (dev->cs_ctl == NULL) || \
        (p_tx == NULL) || \
        (tx_len == 0))
        return false;

    dev->cs_ctl(false);

    state = dev->bus_tx(p_tx, tx_len, W25NXX_BUS_COMMU_TIMEOUT) ? true : false;
    
    if (p_buf && size)
        state &= dev->bus_tx(p_buf, size, W25NXX_BUS_COMMU_TIMEOUT) ? true : false;

    dev->cs_ctl(true);

    return state;
}

static bool DevW25Nxx_Trans_Recevice(DevW25NxxObj_TypeDef *dev, uint8_t *p_tx, uint16_t tx_len, uint8_t *p_rx, uint16_t rx_len)
{
    bool state = false;

    if ((dev == NULL) || \
        (dev->cs_ctl == NULL) || \
        (dev->bus_tx == NULL) || \
        (dev->bus_rx == NULL))
        return false;

    dev->cs_ctl(false);

    if (p_tx && tx_len)
        state = dev->bus_tx(p_tx, tx_len, W25NXX_BUS_COMMU_TIMEOUT) ? true : false;

    if (p_rx && rx_len)
        state &= dev->bus_rx(p_rx, rx_len, W25NXX_BUS_COMMU_TIMEOUT) ? true : false;

    dev->cs_ctl(true);

    return state;
}

static bool DevW25Nxx_Trans_Duplex(DevW25NxxObj_TypeDef *dev, uint8_t *p_tx, uint8_t *p_rx, uint16_t len)
{
    uint16_t trans_out = 0;

    if ((dev == NULL) || \
        (dev->bus_trans == NULL) || \
        (dev->cs_ctl == NULL) || \
        (p_rx == NULL) || \
        (p_tx == NULL) || \
        (len == 0))
        return false;

    dev->cs_ctl(false);
    trans_out = dev->bus_trans(p_tx, p_rx, len, W25NXX_BUS_COMMU_TIMEOUT);
    dev->cs_ctl(true);

    return trans_out ? true : false;
}

static uint32_t DevW25Nxx_Get_Page(DevW25NxxObj_TypeDef *dev, uint32_t addr)
{
    UNUSED(dev);
    return (addr / W25NXX_PAGE_SIZE);
}

static uint32_t DevW25Nxx_Get_Column(uint32_t addr)
{
    return (addr % W25NXX_PAGE_SIZE);
}

static uint32_t DevW25Nxx_Get_Block(uint32_t addr)
{
    return (addr / W25NXX_BLOCK_SIZE);
}

static bool DevW25Nxx_Check_PageAddr(DevW25NxxObj_TypeDef *dev, uint32_t page_addr)
{
    switch ((uint8_t)dev->prod_type)
    {
        case DevW25N_01:
            if (page_addr > W25N01GV_PAGE_NUM)
                return false;
            break;

        default: return false;
    }
    return true;
}

static W25Nxx_FormatAddr_TypeDef DevW25Nxx_FormatAddr(uint32_t colum_addr)
{
    W25Nxx_FormatAddr_TypeDef for_addr;
    memset(&for_addr, 0, sizeof(W25Nxx_FormatAddr_TypeDef));
    
    for_addr.addr_h = ((uint8_t *)&colum_addr)[1];
    for_addr.addr_l = ((uint8_t *)&colum_addr)[0];
    return for_addr;
}

uint8_t rx_tmp[2048];
static DevW25Nxx_Error_List DevW25Nxx_Init(DevW25NxxObj_TypeDef *dev)
{
    DevW25Nxx_Error_List err = DevW25Nxx_Ok;
    uint32_t sys_time = 0;

    if ((dev == NULL) || \
        (dev->delay_ms == NULL) || \
        (dev->systick == NULL))
        return DevW25Nxx_Error;
    
    memset(rx_tmp, 0xAA, sizeof(rx_tmp));

    dev->init_state = false;
    /* check read status */
    if (!DevW25Nxx_Wait(dev, BF_BUSY))
        return DevW25Nxx_Error;
    
    /* soft reset */
    if (!DevW25Nxx_Soft_Reset(dev))
        return DevW25Nxx_Error;

    /* get product id */
    dev->prod_type = DevW25Nxx_Get_ProductID(dev);
    switch ((uint8_t)dev->prod_type)
    {
        case DevW25N_01:
            dev->list_size = (W25N01GV_BLOCK_NUM / 64);
            if (dev->list_size * 2 > (sizeof(dev->bb_list) / sizeof(dev->bb_list[0])))
                return DevW25Nxx_Error;
            
            dev->state_list = (DevW25Nxx_BBItem_TypeDef *)dev->bb_list;
            dev->check_list = (DevW25Nxx_BBItem_TypeDef *)(dev->bb_list + dev->list_size);
            for (uint16_t i = 0; i < dev->list_size; i ++)
            {
                dev->state_list[i].val = 0;
                dev->check_list[i].val = 0;
            }

            W25NXX_INFO(" check bad block\r\n");
            for (uint16_t i = 0; i < W25N01GV_BLOCK_NUM; i ++)
                DevW25Nxx_CheckBlock(dev, i);
            W25NXX_INFO(" Bab block num %d\r\n", dev->bb_cnt);
            break;

        case DevW25N_None:
        default: return DevW25Nxx_Error;
    }

    dev->delay_ms(100);
    dev->init_state = true;

    /* test code */
    uint8_t tx_tmp[] = {'b', 'a', 'd', 'a', 's', 's', ' ', '8', 'b', 'i', 't', ' ', 't', 'e', 's', 't', '\0'};
    if (DevW25Nxx_Erase_PageOnBlock(dev, 0) != DevW25Nxx_Ok)
        W25NXX_INFO(" Erase block error\r\n");

    // if (DevW25Nxx_Write_Page(dev, 0, tx_tmp, sizeof(tx_tmp)) != DevW25Nxx_Ok)
    //     W25NXX_INFO(" Write page failed\r\n");

    for (uint8_t i = 0; i < 2; i++)
    {
        if (DevW25Nxx_Read_Page(dev, i * W25NXX_PAGE_SIZE, rx_tmp, W25NXX_PAGE_SIZE) != DevW25Nxx_Ok)
        {
            W25NXX_INFO(" Read page failed\r\n");
        }
        else
        {
            for (uint8_t i = 0; i < 128; i ++)
            {
                // W25NXX_INFO(" %c %c %c %c %c %c %c %c %c %c %c %c %c %c %c %c\r\n", 
                W25NXX_INFO(" 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r\n", \
                            rx_tmp[i * 16 + 0],  rx_tmp[i * 16 + 1],  rx_tmp[i * 16 + 2],  rx_tmp[i * 16 + 3], \
                            rx_tmp[i * 16 + 4],  rx_tmp[i * 16 + 5],  rx_tmp[i * 16 + 6],  rx_tmp[i * 16 + 7], \
                            rx_tmp[i * 16 + 8],  rx_tmp[i * 16 + 9],  rx_tmp[i * 16 + 10], rx_tmp[i * 16 + 11], \
                            rx_tmp[i * 16 + 12], rx_tmp[i * 16 + 13], rx_tmp[i * 16 + 14], rx_tmp[i * 16 + 15]);
            }
            W25NXX_INFO(" \r\n");
        }
    }
    /* test code */

    W25NXX_INFO(" Init done\r\n");
    return DevW25Nxx_Ok;
}

static bool DevW25Nxx_Soft_Reset(DevW25NxxObj_TypeDef *dev)
{
    uint8_t tx_tmp[2] = {0, 0};
    uint8_t rx_tmp[2] = {0, 0};
    uint8_t field = 0x00;

    if ((dev == NULL) || \
        (dev->cs_ctl == NULL))
        return false;

    W25NXX_INFO(" Resetting\r\n");
    memset(tx_tmp, 0, sizeof(tx_tmp));
    tx_tmp[0] = W25NXX_RESET_CMD;
    if (!DevW25Nxx_Trans_Duplex(dev, tx_tmp, rx_tmp, sizeof(tx_tmp)))
        return false;

    // wait for w25n01 ready
    if (!DevW25Nxx_Wait(dev, BF_BUSY))
        return false;

    // no protection WP-E off
    field |= BF_WPE;
    if (DevW25Nxx_WriteReg_Set(dev, W25NXX_SR0_ADDR, field) != DevW25Nxx_Ok)
    {
        W25NXX_INFO(" WP-E set error\r\n");
        return false;
    }

    // enable buffered read mode (BUF = 1) ECC enabled (ECC = 1)
    field = 0;
    field |= BF_BUF;
    field |= BF_ECC_E;
    if (DevW25Nxx_WriteReg_Set(dev, W25NXX_SR1_ADDR, field) != DevW25Nxx_Ok)
    {
        W25NXX_INFO(" BUF and ECC_E set error\r\n");
        return false;
    }

    W25NXX_INFO(" Reset done\r\n");
    return true;
}

static DevW25Nxx_Error_List DevW25Nxx_Check_ECC(DevW25NxxObj_TypeDef *dev)
{
    DevW25Nxx_SR2_TypeDef sr2;
    
    sr2.val = 0;
    if ((dev == NULL) || \
        (DevW25Nxx_Check_Reg_Status(dev, W25NXX_SR2_ADDR, &sr2.val) != DevW25Nxx_Ok) || \
        sr2.bit.ECC_1)
        return DevW25Nxx_Error;

    return DevW25Nxx_Ok;
}

static bool DevW25Nxx_Wait(DevW25NxxObj_TypeDef *dev, DevW25Nxx_SR2_BitField_TypeDef bit)
{
    uint8_t time_out = 0;
    DevW25Nxx_Error_List state = DevW25Nxx_Busy;
    DevW25Nxx_SR2_TypeDef sr2;

    sr2.val = 0x00;
    if ((dev == NULL) || \
        (dev->delay_ms == NULL))
        return false;

    while (true)
    {
        state = DevW25Nxx_Check_Reg_Status(dev, W25NXX_SR2_ADDR, &sr2.val);

        if (state == DevW25Nxx_Ok)
        {
            if ((bit == BF_BUSY) && (sr2.bit.BUSY == 0))
                return true;

            if ((bit == BF_WEL) && (sr2.bit.WEL == 0))
                return true;
        }

        dev->delay_ms(1);
        if (time_out >= W25NXX_BUS_COMMU_TIMEOUT)
            return false;

        time_out ++;
    }
    
    return false;
}

static DevW25Nxx_ProdType_List DevW25Nxx_Get_ProductID(DevW25NxxObj_TypeDef *dev)
{
    uint8_t tx_tmp[5] = {0};
    uint8_t rx_tmp[5] = {0};

    if ((dev == NULL) || \
        (dev->cs_ctl == NULL))
        return DevW25N_None;

    memset(tx_tmp, 0, sizeof(tx_tmp));
    memset(rx_tmp, 0, sizeof(rx_tmp));
    tx_tmp[0] = W25NXX_JEDEC_ID;

    if (!DevW25Nxx_Trans_Duplex(dev, tx_tmp, rx_tmp, sizeof(rx_tmp)))
        return DevW25N_None;

    ((uint8_t *)&dev->prod_code)[3] = 0;
    ((uint8_t *)&dev->prod_code)[2] = rx_tmp[2];
    ((uint8_t *)&dev->prod_code)[1] = rx_tmp[3];
    ((uint8_t *)&dev->prod_code)[0] = rx_tmp[4];
    W25NXX_INFO(" W25N01 reading ID: 0x%08X\r\n", dev->prod_code);
    switch (dev->prod_code)
    {
        case W25N01GVZEIG_ID: W25NXX_INFO(" W25N01 found\r\n"); return DevW25N_01;
        default: W25NXX_INFO("No W25Nxx serial module found\r\n"); return DevW25N_None;
    }

    return DevW25N_None;
}

static DevW25Nxx_Error_List DevW25Nxx_Check_Reg_Status(DevW25NxxObj_TypeDef *dev, uint8_t reg_addr, uint8_t *reg)
{
    uint8_t cmd[3] = {W25NXX_READ_STATUS_CMD, reg_addr, 0};
    uint8_t reg_val[3] = {0, 0, 0};
    DevW25Nxx_SR2_TypeDef sr2;

    if ((dev == NULL) || \
        (reg == NULL) || \
        !DevW25Nxx_Trans_Duplex(dev, cmd, reg_val, sizeof(cmd)))
        return DevW25Nxx_Read_Status_Error;

    *reg = reg_val[2];
    return DevW25Nxx_Ok;
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

static void DevW25Nxx_Print_Reg(uint8_t reg_addr, uint8_t val)
{
    if (reg_addr == W25NXX_SR0_ADDR)
    {
        DevW25Nxx_SR0_TypeDef SR0_data;
        SR0_data.val = val;

        W25NXX_INFO(" Reg SR0\r\n");
        W25NXX_INFO(" SRP_1  %s\r\n", SR0_data.bit.SRP_1 ? "set" : "reset");
        W25NXX_INFO(" WP-E   %s\r\n", SR0_data.bit.WPE   ? "set" : "reset");
        W25NXX_INFO(" TB     %s\r\n", SR0_data.bit.TB    ? "set" : "reset");
        W25NXX_INFO(" BP_0   %s\r\n", SR0_data.bit.BP_0  ? "set" : "reset");
        W25NXX_INFO(" BP_1   %s\r\n", SR0_data.bit.BP_1  ? "set" : "reset");
        W25NXX_INFO(" BP_2   %s\r\n", SR0_data.bit.BP_2  ? "set" : "reset");
        W25NXX_INFO(" BP_3   %s\r\n", SR0_data.bit.BP_3  ? "set" : "reset");
        W25NXX_INFO(" SRP_0  %s\r\n", SR0_data.bit.SRP_0 ? "set" : "reset");

        return;
    }
    else if (reg_addr == W25NXX_SR1_ADDR)
    {
        DevW25Nxx_SR1_TypeDef SR1_data;
        SR1_data.val = val;

        W25NXX_INFO(" Reg SR1\r\n");
        W25NXX_INFO(" BUF   %s\r\n", SR1_data.bit.BUF ? "set" : "reset");
        W25NXX_INFO(" ECC_E %s\r\n", SR1_data.bit.ECC_E ? "set" : "reset");
        W25NXX_INFO(" SR1_L %s\r\n", SR1_data.bit.SR1_L ? "set" : "reset");
        W25NXX_INFO(" OTP_E %s\r\n", SR1_data.bit.OTP_E ? "set" : "reset");
        W25NXX_INFO(" OTP_L %s\r\n", SR1_data.bit.OTP_L ? "set" : "reset");
        return;
    }
    W25NXX_INFO(" \r\n");

    W25NXX_INFO(" Unknow reg\r\n");
}

static DevW25Nxx_Error_List DevW25Nxx_WriteReg_Set(DevW25NxxObj_TypeDef *dev, uint8_t reg_addr, uint8_t field)
{
    uint8_t tx[3] = {W25NXX_WRITE_STATUS_CMD, reg_addr};
    uint8_t rx[3] = {0, 0, 0};
    uint8_t set_val = 0;

    if ((dev == NULL) || \
        ((reg_addr != W25NXX_SR0_ADDR) && \
         (reg_addr != W25NXX_SR1_ADDR)))
        return DevW25Nxx_Error;

    if (reg_addr == W25NXX_SR0_ADDR)
    {
        W25NXX_INFO(" Set SR0\r\n");

        if (field & BF_SRP_1)
            W25NXX_SET_SINGLE_BIT(ConvertToSR0_RegFormat(&set_val)->bit.SRP_1);
        
        if (field & BF_WPE) 
            W25NXX_SET_SINGLE_BIT(ConvertToSR0_RegFormat(&set_val)->bit.WPE);

        if (field & BF_TB)
            W25NXX_SET_SINGLE_BIT(ConvertToSR0_RegFormat(&set_val)->bit.TB);

        if (field & BF_BP_0)
            W25NXX_SET_SINGLE_BIT(ConvertToSR0_RegFormat(&set_val)->bit.BP_0);

        if (field & BF_BP_1)
            W25NXX_SET_SINGLE_BIT(ConvertToSR0_RegFormat(&set_val)->bit.BP_1);

        if (field & BF_BP_2)
            W25NXX_SET_SINGLE_BIT(ConvertToSR0_RegFormat(&set_val)->bit.BP_2);

        if (field & BF_BP_3)
            W25NXX_SET_SINGLE_BIT(ConvertToSR0_RegFormat(&set_val)->bit.BP_3);

        if (field & BF_SRP_0)
            W25NXX_SET_SINGLE_BIT(ConvertToSR0_RegFormat(&set_val)->bit.SRP_0);
    }
    else if (reg_addr == W25NXX_SR1_ADDR)
    {
        W25NXX_INFO(" Set SR1\r\n");
        
        if (field & BF_BUF)
            W25NXX_SET_SINGLE_BIT(ConvertToSR1_RegFormat(&set_val)->bit.BUF);
        
        if (field & BF_ECC_E)
            W25NXX_SET_SINGLE_BIT(ConvertToSR1_RegFormat(&set_val)->bit.ECC_E);

        if (field & BF_SR1_L)
            W25NXX_SET_SINGLE_BIT(ConvertToSR1_RegFormat(&set_val)->bit.SR1_L);
        
        if (field & BF_OTP_E)
            W25NXX_SET_SINGLE_BIT(ConvertToSR1_RegFormat(&set_val)->bit.OTP_E);
        
        if (field & BF_OTP_L)
            W25NXX_SET_SINGLE_BIT(ConvertToSR1_RegFormat(&set_val)->bit.OTP_L);
    }
    
    tx[0] = W25NXX_WRITE_STATUS_CMD;
    tx[2] = set_val;
    if (!DevW25Nxx_Trans_Duplex(dev, tx, rx, sizeof(tx)))
    {
        W25NXX_INFO(" Failed on reg setting\r\n");
        return DevW25Nxx_Error;
    }

    if (!DevW25Nxx_Wait(dev, BF_BUSY))
        return DevW25Nxx_Error;

    tx[0] = W25NXX_READ_STATUS_CMD;
    tx[2] = 0;
    if (!DevW25Nxx_Trans_Duplex(dev, tx, rx, sizeof(tx)))
        return DevW25Nxx_Error;

    if (set_val != rx[2])
    {
        W25NXX_INFO(" set val\r\n");
        DevW25Nxx_Print_Reg(reg_addr, set_val);
        W25NXX_INFO(" reg val\r\n");
        DevW25Nxx_Print_Reg(reg_addr, rx[2]);
        return DevW25Nxx_Error;
    }

    W25NXX_INFO(" Reg set successed\r\n");
    return DevW25Nxx_Ok;
}

static DevW25Nxx_Error_List DevW25Nxx_Send_CMD(DevW25NxxObj_TypeDef *dev, uint8_t reg, uint32_t page_addr)
{
    uint8_t tx_tmp[4] = {reg, 0, 0, 0};
    uint8_t rx_tmp[4] = {0, 0, 0, 0};

    if (dev == NULL)
        return DevW25Nxx_Error;

    tx_tmp[2] = ((uint8_t *)&page_addr)[1];
    tx_tmp[3] = ((uint8_t *)&page_addr)[0];

    if (!DevW25Nxx_Trans_Duplex(dev, tx_tmp, rx_tmp, sizeof(tx_tmp)))
        return DevW25Nxx_Send_Command_Error;

    return DevW25Nxx_Ok;
}

/****************************************************************** block check section ***************************************************************************/
static DevW25Nxx_Error_List DevW25Nxx_Read_ExtensionOnBlock(DevW25NxxObj_TypeDef *dev, uint32_t addr, uint8_t *p_buf, uint32_t size)
{
    W25Nxx_FormatAddr_TypeDef for_addr; 
    uint8_t tx_tmp[4] = {W25NXX_READ};
    uint32_t page_addr = DevW25Nxx_Get_Page(dev, addr);

    if ((dev == NULL) || (p_buf == NULL) || (size == 0) || (size > W25NXX_EXT_DATA_SIZE))
        return DevW25Nxx_Error;

    memset(&for_addr, 0, sizeof(for_addr));
    
    /* check read status */
    if (!DevW25Nxx_Wait(dev, BF_BUSY))
        return DevW25Nxx_Error;

    /* send read command */
    if ((DevW25Nxx_Send_CMD(dev, W25NXX_PAGE_DATA_READ, page_addr) != DevW25Nxx_Ok) || \
        (DevW25Nxx_ReadDataBuffer(dev, W25NXX_EXT_DATA_COLUMN, p_buf, size) != DevW25Nxx_Error))
        return DevW25Nxx_Error;

    return DevW25Nxx_Ok;
}

static DevW25Nxx_Error_List DevW25Nxx_CheckBlock(DevW25NxxObj_TypeDef *dev, uint32_t block_index)
{
    uint16_t list_i = 0;
    uint8_t bit_map_i = 0;
    uint64_t check_bit = 0;
    uint32_t block_sum = 0;
    bool bb_found = false;
    uint8_t ECC_buf[W25NXX_EXT_DATA_SIZE];
    uint32_t addr = block_index * W25NXX_BLOCK_SIZE;

    memset(ECC_buf, 0, W25NXX_EXT_DATA_SIZE);
    if (dev == NULL)
        return DevW25Nxx_Error;

    list_i = block_index / 64;
    bit_map_i = (block_index % 64) / 8;
    check_bit = 1 << (block_index % 64) % 8;

    /* check list first if block been none checked then update list */
    if (dev->list_size && ((dev->check_list[list_i].bit_map_list[bit_map_i] & check_bit) == 0))
        dev->check_list[list_i].bit_map_list[bit_map_i] |= check_bit;

    /* read extension data */
    if (!DevW25Nxx_Read_ExtensionOnBlock(dev, addr, ECC_buf, W25NXX_EXT_DATA_SIZE) != DevW25Nxx_Ok)
        return DevW25Nxx_Error;

    /* the first byte on ECC arse is bad block marker */
    if (ECC_buf[0] != W25NXX_BLOCK_VALID_TAG)
    {
        dev->bb_cnt ++;
        bb_found = true;
    }

    /* update check and state list */
    if (dev->list_size && bb_found)
        dev->state_list[list_i].bit_map_list[bit_map_i] |= check_bit;

    return DevW25Nxx_Ok;
}

/****************************************************************** write section ***************************************************************************/
static DevW25Nxx_Error_List DevW25Nxx_WriteEn(DevW25NxxObj_TypeDef *dev, bool en)
{
    uint8_t cmd[2] = {W25NXX_WRITE_DISABLE, 0};

    if (en)
        cmd[0] = W25NXX_WRITE_ENABLE;

    if ((dev == NULL) || \
        !DevW25Nxx_Trans_Duplex(dev, &cmd[0], &cmd[1], sizeof(uint8_t)))
        return DevW25Nxx_Error;

    return DevW25Nxx_Ok;
}

static DevW25Nxx_Error_List DevW25Nxx_LoadData(DevW25NxxObj_TypeDef *dev, uint32_t column_addr, uint8_t *p_buf, uint16_t len)
{
    uint8_t tx_tmp[3] = {W25NXX_PROGRAM_DATA_LOAD, 0, 0};
    W25Nxx_FormatAddr_TypeDef for_addr;

    if ((dev == NULL) || (p_buf == NULL))
        return DevW25Nxx_Error;

    memset(&for_addr, 0, sizeof(W25Nxx_FormatAddr_TypeDef));

    if (DevW25Nxx_WriteEn(dev, true) != DevW25Nxx_Ok)
        return DevW25Nxx_Error;

    for_addr = DevW25Nxx_FormatAddr(DevW25Nxx_Get_Column(column_addr));

    tx_tmp[1] = for_addr.addr_h;
    tx_tmp[2] = for_addr.addr_l;

    if (!DevW25Nxx_Trans_Buff(dev, tx_tmp, sizeof(tx_tmp), p_buf, len))
        return DevW25Nxx_Error;

    return DevW25Nxx_Ok;
}

static DevW25Nxx_Error_List DevW25Nxx_ProgramExexute(DevW25NxxObj_TypeDef *dev, uint32_t page_addr)
{
    DevW25Nxx_SR2_TypeDef sr2;
    
    /* check bad block first */
    // if ()

    sr2.val = 0;

    if ((dev == NULL) || \
        !DevW25Nxx_Wait(dev, BF_BUSY) || \
        (DevW25Nxx_Send_CMD(dev, W25NXX_PROGRAM_EXECUTE, page_addr) != DevW25Nxx_Ok) || \
        (DevW25Nxx_Check_Reg_Status(dev, W25NXX_SR2_ADDR, &sr2.val) != DevW25Nxx_Ok) || \
        sr2.bit.P_FAIL || \
        !DevW25Nxx_Wait(dev, BF_WEL))
        return DevW25Nxx_Error;

    return DevW25Nxx_Ok;
}

static DevW25Nxx_Error_List DevW25Nxx_Write_Page(DevW25NxxObj_TypeDef *dev, uint32_t addr, uint8_t *p_data, uint32_t size)
{
    uint16_t write_cnt = DevW25Nxx_Get_Page(dev, addr + size) - DevW25Nxx_Get_Page(dev, addr) + 1;
    uint32_t page_addr = addr;
    uint32_t write_size = 0;
    uint32_t write_remain = size;
    uint32_t write_addr = 90;
    uint8_t *p_data_tmp = p_data;
    uint32_t column_addr = 0;

    if ((dev == NULL) || \
        !dev->init_state || \
        (p_data == NULL) || \
        (size == 0))
        return DevW25Nxx_Error;

    if (!DevW25Nxx_Check_PageAddr(dev, DevW25Nxx_Get_Page(dev, addr)) || \
        !DevW25Nxx_Check_PageAddr(dev, DevW25Nxx_Get_Page(dev, addr + size)))
        return DevW25Nxx_Error;

    W25NXX_INFO(" write count %d\r\n", write_cnt);

    /* DO NOT WRITE DATA INTO SPARE AREA !!! */
    for (uint16_t i = 0; i < write_cnt; i ++)
    {
        column_addr = DevW25Nxx_Get_Column(page_addr);
        write_addr = DevW25Nxx_Get_Page(dev, page_addr);

        write_size = write_remain;
        if (write_size >= W25NXX_PAGE_SIZE)
            write_size = W25NXX_PAGE_SIZE;
        
        if (column_addr + write_size > W25NXX_PAGE_SIZE)
            write_size -= (column_addr + write_size - W25NXX_PAGE_SIZE);

        if (write_size == 0)
            return DevW25Nxx_Error;

        if ((DevW25Nxx_LoadData(dev, column_addr, p_data_tmp, write_size) != DevW25Nxx_Ok) || \
            (DevW25Nxx_ProgramExexute(dev, write_addr) != DevW25Nxx_Ok))
            return DevW25Nxx_Error;

        write_remain -= write_size;
        if (write_remain == 0)
            break;

        p_data_tmp += write_size;
        page_addr += W25NXX_PAGE_SIZE;
        column_addr = 0;
    }

    return DevW25Nxx_Ok;
}

/****************************************************************** erase section **************************************************************************/
static DevW25Nxx_Error_List DevW25Nxx_Erase_PageOnBlock(DevW25NxxObj_TypeDef *dev, uint32_t addr)
{
    uint8_t tx_tmp[4] = {W25NXX_BLOCK_ERASE, 0, 0, 0};
    uint8_t rx_tmp[4] = {0};
    uint32_t page_addr = DevW25Nxx_Get_Page(dev, addr);
    DevW25Nxx_SR2_TypeDef sr2;

    sr2.val = 0;
    if ((dev == NULL) || \
        (DevW25Nxx_WriteEn(dev, true) != DevW25Nxx_Ok))
        return DevW25Nxx_Error;

    tx_tmp[2] = ((uint8_t *)&page_addr)[1];
    tx_tmp[3] = ((uint8_t *)&page_addr)[0];
    if (!DevW25Nxx_Trans_Duplex(dev, tx_tmp, rx_tmp, sizeof(tx_tmp)) || \
        (DevW25Nxx_Check_Reg_Status(dev, W25NXX_SR2_ADDR, &sr2.val) != DevW25Nxx_Ok) || \
        sr2.bit.E_FAIL || \
        !DevW25Nxx_Wait(dev, BF_WEL))
        return DevW25Nxx_Error;

    return DevW25Nxx_Ok;
}

/****************************************************************** read section ***************************************************************************/
static DevW25Nxx_Error_List DevW25Nxx_ReadDataBuffer(DevW25NxxObj_TypeDef *dev, uint32_t colum_addr, uint8_t *p_buf, uint32_t size)
{
    W25Nxx_FormatAddr_TypeDef for_addr; 
    uint8_t tx_tmp[4] = {W25NXX_READ};

    memset(&for_addr, 0, sizeof(for_addr));
    if ((dev == NULL) || \
        (p_buf == NULL) || \
        (size == 0) || \
        !DevW25Nxx_Wait(dev, BF_BUSY))
        return DevW25Nxx_Error;

    for_addr = DevW25Nxx_FormatAddr(colum_addr);
    tx_tmp[1] = for_addr.addr_h;
    tx_tmp[2] = for_addr.addr_l;
    tx_tmp[3] = 0x00;

    if (DevW25Nxx_Trans_Recevice(dev, tx_tmp, sizeof(tx_tmp), p_buf, size))
        return DevW25Nxx_Ok;

    return DevW25Nxx_Error;
}

static DevW25Nxx_Error_List DevW25Nxx_Read_Page(DevW25NxxObj_TypeDef *dev, uint32_t addr, uint8_t *p_data, uint32_t size)
{
    uint32_t page_addr = addr;
    uint32_t read_addr = 0;
    uint32_t column_addr = 0;
    uint16_t read_cnt = DevW25Nxx_Get_Page(dev, addr + size) - DevW25Nxx_Get_Page(dev, addr) + 1;
    uint8_t *p_data_tmp = p_data;
    uint32_t read_size = size;
    uint32_t read_remain = size;
    uint32_t c_block = 0;   /* current block */
    uint32_t l_block = 0;   /* last block */
    uint8_t cmd[4];
    DevW25Nxx_SR2_TypeDef sr2;

    memset(cmd, 0, sizeof(cmd));
    sr2.val = 0;

    if ((dev == NULL) || \
        !dev->init_state || \
        (p_data == NULL) || \
        (dev->systick == NULL) || \
        (dev->delay_ms == NULL))
        return DevW25Nxx_Error;

    if (!DevW25Nxx_Check_PageAddr(dev, DevW25Nxx_Get_Page(dev, addr)) || \
        !DevW25Nxx_Check_PageAddr(dev, DevW25Nxx_Get_Page(dev, addr + size)))
        return DevW25Nxx_Error;

    for (uint8_t i = 0; i < read_cnt; i++)
    {
        column_addr = DevW25Nxx_Get_Column(page_addr);
        read_addr = DevW25Nxx_Get_Page(dev, page_addr);
        c_block = DevW25Nxx_Get_Block(read_addr);

        read_size = read_remain;
        if (read_size >= W25NXX_PAGE_SIZE)
            read_size = W25NXX_PAGE_SIZE;
        
        if (column_addr + read_size > W25NXX_PAGE_SIZE)
            read_size -= (column_addr + read_size - W25NXX_PAGE_SIZE);

        if (read_size == 0)
            return DevW25Nxx_Error;
        
        /* send target page read cmd */
        /* read buffer data */
        if ((DevW25Nxx_Send_CMD(dev, W25NXX_PAGE_DATA_READ, read_addr) != DevW25Nxx_Ok) || \
            (DevW25Nxx_ReadDataBuffer(dev, column_addr, p_data_tmp, read_size) != DevW25Nxx_Ok))
            return DevW25Nxx_Error;
        
        /* after read, shift page address and read buffer address */
        read_remain -= read_size;
        if (read_remain == 0)
            break;
        
        p_data_tmp += read_size;
        page_addr += W25NXX_PAGE_SIZE;
        column_addr = 0;
    
        /* if block changed after page read check ECC */
        if ((i == 0) || (l_block != c_block))
        {
            if (DevW25Nxx_Check_Reg_Status(dev, W25NXX_SR2_ADDR, &sr2.val) != DevW25Nxx_Ok)
                return DevW25Nxx_Error;

            /* still in developping */
            if (sr2.bit.ECC_1)
            {
                /* update bad block */
                return DevW25Nxx_ECC_Error;
            }
        }

        l_block = c_block;
    }

    return DevW25Nxx_Ok;
}

