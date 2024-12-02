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
#define ConvertPageFormat(x)        ((x / W25NXX_PAGE_PRE_BLOCK) << 6) | (x %  W25NXX_PAGE_PRE_BLOCK)
#define ConvertToSR0_RegFormat(x)   ((DevW25Nxx_SR0_TypeDef *)x)
#define ConvertToSR1_RegFormat(x)   ((DevW25Nxx_SR1_TypeDef *)x)
#define ConvertToSR2_RegFormat(x)   ((DevW25Nxx_SR2_TypeDef *)x)

typedef struct
{
    uint16_t p_addr;    /* physical address */
    uint16_t l_addr;    /* logic address */
} W25Nxx_BBLUT_TypeDef;

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
static bool DevW25Nxx_Trans_Duplex(DevW25NxxObj_TypeDef *dev, uint8_t *p_tx, uint8_t *p_rx, uint16_t len);
static DevW25Nxx_ProdType_List DevW25Nxx_Get_ProductID(DevW25NxxObj_TypeDef *dev);
static bool DevW25Nxx_Soft_Reset(DevW25NxxObj_TypeDef *dev);
static DevW25Nxx_Error_List DevW25Nxx_Check_Read_Status(DevW25NxxObj_TypeDef *dev, uint8_t reg_addr, uint8_t *reg);
static DevW25Nxx_Error_List DevW25Nxx_WriteEn(DevW25NxxObj_TypeDef *dev, bool en);
static DevW25Nxx_Error_List DevW25Nxx_WriteReg_Set(DevW25NxxObj_TypeDef *dev, uint8_t reg_addr, uint8_t field);
static bool DevW25Nxx_Wait_Busy(DevW25NxxObj_TypeDef *dev);
static DevW25Nxx_Error_List DevW25Nxx_Send_CMD(DevW25NxxObj_TypeDef *dev, uint8_t cmd, uint32_t addr);

/* external function */
static DevW25Nxx_Error_List DevW25Nxx_Init(DevW25NxxObj_TypeDef *dev);
static DevW25Nxx_DeviceInfo_TypeDef DevW25Nxx_Get_Info(DevW25NxxObj_TypeDef *dev);
static uint32_t DevW25Nxx_Get_Page(DevW25NxxObj_TypeDef *dev, uint32_t addr);
static DevW25Nxx_Error_List DevW25Nxx_Read_Page(DevW25NxxObj_TypeDef *dev, uint32_t addr, uint8_t *p_data, uint32_t size);

DevW25Nxx_TypeDef DevW25Nxx = {
    .init = DevW25Nxx_Init,
    .info = DevW25Nxx_Get_Info,
    .get_page = DevW25Nxx_Get_Page,
    // .read = ,
};

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

static DevW25Nxx_Error_List DevW25Nxx_Init(DevW25NxxObj_TypeDef *dev)
{
    DevW25Nxx_Error_List err = DevW25Nxx_Ok;
    uint32_t sys_time = 0;

    if ((dev == NULL) || \
        (dev->delay_ms == NULL) || \
        (dev->systick == NULL))
        return DevW25Nxx_Error;
    
    dev->init_state = false;
    dev->write_en = false;
    dev->cur_addr = UINT32_MAX;

    /* check read status */
    if (!DevW25Nxx_Wait_Busy(dev))
        return DevW25Nxx_Error;
    
    /* soft reset */
    if (!DevW25Nxx_Soft_Reset(dev))
        return DevW25Nxx_Error;

    /* get product id */
    dev->prod_type = DevW25Nxx_Get_ProductID(dev);
    if (dev->prod_type == DevW25N_None)
        return DevW25Nxx_Error;

    /* disable write protect */
    if (DevW25Nxx_WriteEn(dev, true) != DevW25Nxx_Ok)
        return DevW25Nxx_Error;

    dev->delay_ms(100);
    dev->init_state = true;
    dev->cur_addr = 0;

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
    if (!DevW25Nxx_Wait_Busy(dev))
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

static bool DevW25Nxx_Wait_Busy(DevW25NxxObj_TypeDef *dev)
{
    uint8_t time_out = 0;
    DevW25Nxx_Error_List state = DevW25Nxx_Busy;
    DevW25Nxx_SR2_TypeDef sr2;

    sr2.val = 0x00;
    if ((dev == NULL) || \
        (dev->delay_ms == NULL))
        return false;

    while (state == DevW25Nxx_Busy)
    {
        state = DevW25Nxx_Check_Read_Status(dev, W25NXX_SR2_ADDR, &sr2.val);

        if ((state == DevW25Nxx_Ok) && (sr2.bit.BUSY == 0))
            return true;

        dev->delay_ms(1);
        if (time_out >= W25NXX_BUS_COMMU_TIMEOUT)
            return false;

        time_out ++;
    }
    
    W25NXX_INFO(" Device busy\r\n");
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

static DevW25Nxx_Error_List DevW25Nxx_Check_Read_Status(DevW25NxxObj_TypeDef *dev, uint8_t reg_addr, uint8_t *reg)
{
    uint8_t cmd[3] = {W25NXX_READ_STATUS_CMD, reg_addr, 0};
    uint8_t reg_val[3] = {0, 0, 0};
    DevW25Nxx_SR2_TypeDef sr2;

    if ((dev == NULL) || \
        (reg == NULL) || \
        !DevW25Nxx_Trans_Duplex(dev, cmd, reg_val, sizeof(cmd)))
        return DevW25Nxx_Error;

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

/* still in developping */
/* return false when current block is bad block else return true */
static DevW25Nxx_BlockState_TypeDef DevW25Nxx_Check_CurBlock(DevW25NxxObj_TypeDef *dev, uint32_t black_addr)
{
    if (dev == NULL)
        return Block_Invalid;

    /* read first page in selected block */
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

    if (!DevW25Nxx_Wait_Busy(dev))
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

static DevW25Nxx_Error_List DevW25Nxx_WriteEn(DevW25NxxObj_TypeDef *dev, bool en)
{
    uint8_t cmd[2] = {W25NXX_WRITE_DISABLE, 0};
    uint8_t dummy[2] = {0, 0};

    if (en)
        cmd[0] = W25NXX_WRITE_ENABLE;

    if ((dev == NULL) || \
        !DevW25Nxx_Trans_Duplex(dev, cmd, dummy, sizeof(cmd)))
        return DevW25Nxx_Error;

    dev->write_en = en;
    return DevW25Nxx_Ok;
}

static DevW25Nxx_Error_List DevW25Nxx_Write_Page(DevW25NxxObj_TypeDef *dev, uint16_t page, uint8_t *p_data, uint16_t len)
{
    uint8_t cmd[3];
    DevW25Nxx_SR0_TypeDef sr0;

    sr0.val = 0;
    memset(cmd, 0, sizeof(cmd));
    if ((dev == NULL) || \
        !dev->init_state || \
        (p_data == NULL) || \
        (len == 0))
        return DevW25Nxx_Error;

    /* check write protect status */
    cmd[0] = W25NXX_READ_STATUS_CMD;
    cmd[1] = W25NXX_SR0_ADDR;

    if (!DevW25Nxx_Trans_Receive(dev, cmd, 2, &sr0.val, 1))
        return DevW25Nxx_Error;

    if (sr0.bit.WPE)
    {
        /* clear write protect */
        sr0.bit.WPE = false;
        cmd[2] = sr0.val;
        if (!DevW25Nxx_Write(dev, cmd, sizeof(cmd)))
            return DevW25Nxx_Error;
    }

    /* set write enable */
    if (DevW25Nxx_WriteEn(dev, true) == DevW25Nxx_Error)
        return DevW25Nxx_Error;

    return DevW25Nxx_Ok;
}

static DevW25Nxx_Error_List DevW25Nxx_Send_CMD(DevW25NxxObj_TypeDef *dev, uint8_t cmd, uint32_t addr)
{
    uint32_t page = 0;
    uint8_t buf[4];

    memset(cmd, 0, sizeof(cmd));

    if (dev == NULL)
        return DevW25Nxx_Error;

    page = DevW25Nxx_Get_Page(dev, addr);
    buf[0] = cmd;
    buf[1] = 0x00;
    buf[2] = (page >> 8) & 0xFF;
    buf[3] = (page >> 0) & 0xFF;

    return DevW25Nxx_Write(dev, buf, sizeof(buf)) ? DevW25Nxx_Ok : DevW25Nxx_Error;
}

static DevW25Nxx_Error_List DevW25Nxx_Read_PageOnBlock(DevW25NxxObj_TypeDef *dev, uint32_t addr, uint8_t *p_data, uint32_t size)
{
    DevW25Nxx_Error_List err = DevW25Nxx_Ok;
    uint32_t sys_time = 0;
    uint32_t start_page = 0;
    uint16_t read_num = size / W25NXX_PAGE_SIZE;
    uint8_t cmd[4];
    uint16_t state = false;
    uint16_t offset = 0;

    memset(cmd, 0, sizeof(cmd));

    if ((dev == NULL) || \
        !dev->init_state || \
        (p_data == NULL) || \
        (dev->systick == NULL) || \
        (dev->delay_ms == NULL) || \
        (size < (W25NXX_PAGE_SIZE + W25N0GV_ECC_INFO_SIZE)))
        return DevW25Nxx_Error;

    for (uint8_t i = 0; i < read_num; i++)
    {
        start_page = DevW25Nxx_Get_Page(dev, addr);
        if (!DevW25Nxx_Wait_Busy(dev))
            break;
        
        // static DevW25Nxx_Error_List DevW25Nxx_Send_CMD(DevW25NxxObj_TypeDef *dev, uint8_t cmd, uint32_t addr)
    }

    return DevW25Nxx_Error;
}

static DevW25Nxx_Error_List DevW25Nxx_Read_ExtensionOnBlock(DevW25NxxObj_TypeDef *dev, uint32_t addr, uint8_t *p_data, uint32_t size)
{
    DevW25Nxx_Error_List err;
    uint32_t sys_time = 0;
    uint8_t cmd[4];

    if ((dev == NULL) || (p_data == NULL) || (size == 0) || (size > 64))
        return DevW25Nxx_Error;

    cmd[0] = W25NXX_READ;
    cmd[1] = (W25NXX_EXTENSION_DATA_INDEX >> 8) & 0xFF;
    cmd[2] = (W25NXX_EXTENSION_DATA_INDEX >> 0) & 0xFF;
    cmd[3] = 0;

    /* check read status */
    if (!DevW25Nxx_Wait_Busy(dev))
        return DevW25Nxx_Error;

    return DevW25Nxx_Trans_Duplex(dev, cmd, p_data, size) ? DevW25Nxx_Ok : DevW25Nxx_Error;
}

/* still in developping */
static DevW25Nxx_Error_List DevW25Nxx_Check_ECC(DevW25NxxObj_TypeDef *dev, uint32_t addr)
{
    if (dev == NULL)
        return DevW25Nxx_Error;
}
