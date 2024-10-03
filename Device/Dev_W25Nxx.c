#include "Dev_W25Nxx.h"

/* test code */
#include "HW_Def.h"
#include "debug_util.h"

#define W25NXX_TAG "[ W25NXX INFO ] "
#define W25NXX_INFO(fmt, ...) Debug_Print(&DebugPort, W25NXX_TAG, fmt, ##__VA_ARGS__)
/* test code */

#define W25NXX_BUS_COMMU_TIMEOUT    100 /* unit: ms */
#define ConvertPageFormat(x)        ((x / W25NXX_PAGE_PRE_BLOCK) << 6) | (x %  W25NXX_PAGE_PRE_BLOCK)
#define ConvertToSR0_RegFormat(x)   ((DevW25Nxx_SR0_TypeDef *)x)
#define ConvertToSR1_RegFormat(x)   ((DevW25Nxx_SR1_TypeDef *)x)
#define ConvertToSR2_RegFormat(x)   ((DevW25Nxx_SR2_TypeDef *)x)

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
static bool DevW25Nxx_Write(DevW25NxxObj_TypeDef *dev, uint8_t *p_tx, uint16_t len);
static bool DevW25Nxx_Read(DevW25NxxObj_TypeDef *dev, uint8_t *p_rx, uint16_t len);
static bool DevW25Nxx_Trans_Receive(DevW25NxxObj_TypeDef *dev, uint8_t *p_tx, uint16_t tx_len, uint8_t *p_rx, uint16_t rx_len);
static bool DevW25Nxx_Trans_Duplex(DevW25NxxObj_TypeDef *dev, uint8_t *p_tx, uint8_t *p_rx, uint16_t len);
static DevW25Nxx_ProdType_List DevW25Nxx_Get_ProductID(DevW25NxxObj_TypeDef *dev);
static bool DevW25Nxx_Soft_Reset(DevW25NxxObj_TypeDef *dev);
static DevW25Nxx_Error_List DevW25Nxx_Check_Read_Status(DevW25NxxObj_TypeDef *dev);
static DevW25Nxx_Error_List DevW25Nxx_WriteEn(DevW25NxxObj_TypeDef *dev);
static DevW25Nxx_Error_List DevW25Nxx_WriteReg_Set(DevW25NxxObj_TypeDef *dev, uint8_t reg_addr, uint8_t field_index, uint8_t val);
static DevW25Nxx_Error_List DevW25Nxx_BadBlock_Managemnet(DevW25NxxObj_TypeDef *dev);

/* external function */
static DevW25Nxx_Error_List DevW25Nxx_Init(DevW25NxxObj_TypeDef *dev);
static DevW25Nxx_DeviceInfo_TypeDef DevW25Nxx_Get_Info(DevW25NxxObj_TypeDef *dev);
static uint32_t DevW25Nxx_Get_Page(DevW25NxxObj_TypeDef *dev, uint32_t addr);
static DevW25Nxx_Error_List DevW25Nxx_Read_Page(DevW25NxxObj_TypeDef *dev, uint32_t addr, uint8_t *p_data, uint32_t size);

DevW25Nxx_TypeDef DevW25Nxx = {
    .init = DevW25Nxx_Init,
    .info = DevW25Nxx_Get_Info,
    .get_page = DevW25Nxx_Get_Page,
    .read = DevW25Nxx_Read_Page,
};

static bool DevW25Nxx_Write(DevW25NxxObj_TypeDef *dev, uint8_t *p_tx, uint16_t len)
{
    uint16_t tx_out = 0;

    if ((dev == NULL) || \
        (dev->bus_tx == NULL) || \
        (dev->cs_ctl == NULL) || \
        (p_tx == NULL) || \
        (len == 0))
        return false;

    dev->cs_ctl(false);
    tx_out = dev->bus_tx(p_tx, len, W25NXX_BUS_COMMU_TIMEOUT);
    dev->cs_ctl(true);

    return tx_out ? true : false;
}

static bool DevW25Nxx_Read(DevW25NxxObj_TypeDef *dev, uint8_t *p_rx, uint16_t len)
{
    uint16_t rx_out = 0;

    if ((dev == NULL) || \
        (dev->bus_rx == NULL) || \
        (dev->cs_ctl) || \
        (p_rx == NULL) || \
        (len == 0))
        return false;

    dev->cs_ctl(false);
    rx_out = dev->bus_rx(p_rx, len, W25NXX_BUS_COMMU_TIMEOUT);
    dev->cs_ctl(true);

    return rx_out ? true : false;
}

static bool DevW25Nxx_Trans_Receive(DevW25NxxObj_TypeDef *dev, uint8_t *p_tx, uint16_t tx_len, uint8_t *p_rx, uint16_t rx_len)
{
    uint16_t tx_out = 0;
    uint16_t rx_out = 0;

    if ((dev == NULL) || \
        (dev->bus_rx == NULL) || \
        (dev->bus_tx == NULL) || \
        (dev->cs_ctl) || \
        (p_rx == NULL) || \
        (p_tx == NULL) || \
        (tx_len == 0) || \
        (rx_len == 0))
        return false;

    dev->cs_ctl(false);
    tx_out = dev->bus_tx(p_tx, tx_len, W25NXX_BUS_COMMU_TIMEOUT);
    rx_out = dev->bus_rx(p_rx, rx_len, W25NXX_BUS_COMMU_TIMEOUT);
    dev->cs_ctl(true);

    if ((tx_out == 0) || \
        (rx_out == 0))
        return false;

    return true;
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
            return DevW25Nxx_TimeOut;

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

    /* bad block management */
    if (DevW25Nxx_BadBlock_Managemnet(dev) != DevW25Nxx_Ok)
        return DevW25Nxx_Error;

    return DevW25Nxx_Ok;
}

static bool DevW25Nxx_Soft_Reset(DevW25NxxObj_TypeDef *dev)
{
    uint8_t tx_tmp[2] = {0};

    if ((dev == NULL) || \
        (dev->cs_ctl == NULL))
        return false;

    memset(tx_tmp, 0, sizeof(tx_tmp));
    tx_tmp[0] = W25NXX_RESET_CMD;
    return DevW25Nxx_Write(dev, tx_tmp, sizeof(tx_tmp));
}

static DevW25Nxx_ProdType_List DevW25Nxx_Get_ProductID(DevW25NxxObj_TypeDef *dev)
{
    uint8_t tx_tmp[2] = {0};
    uint8_t rx_tmp[3] = {0};

    if ((dev == NULL) || \
        (dev->cs_ctl == NULL))
        return DevW25N_None;

    memset(tx_tmp, 0, sizeof(tx_tmp));
    memset(rx_tmp, 0, sizeof(rx_tmp));
    tx_tmp[0] = W25NXX_JEDEC_ID;

    if (!DevW25Nxx_Trans_Receive(dev, tx_tmp, sizeof(tx_tmp), rx_tmp, sizeof(rx_tmp)))
        return DevW25N_None;

    ((uint8_t *)&dev->prod_code)[3] = 0;
    ((uint8_t *)&dev->prod_code)[2] = rx_tmp[0];
    ((uint8_t *)&dev->prod_code)[1] = rx_tmp[1];
    ((uint8_t *)&dev->prod_code)[0] = rx_tmp[2];
    
    switch (dev->prod_code)
    {
        case W25N01GVZEIG_ID: return DevW25N_01;
        default: return DevW25N_None;
    }

    return DevW25N_None;
}

static DevW25Nxx_Error_List DevW25Nxx_Check_Read_Status(DevW25NxxObj_TypeDef *dev)
{
    uint8_t cmd[3] = {W25NXX_READ_STATUS_CMD, W25NXX_SR2_ADDR, 0};
    uint8_t reg_val[3] = {0, 0, 0};
    DevW25Nxx_SR2_TypeDef sr2;

    if ((dev == NULL) || \
        !DevW25Nxx_Trans_Duplex(dev, cmd, reg_val, sizeof(cmd)))
        return DevW25Nxx_Error;

    sr2.val = reg_val[2];
    if (sr2.bit.BUSY == 0)
        return DevW25Nxx_Ok;

    return DevW25Nxx_Busy;
}

static DevW25Nxx_Error_List DevW25Nxx_BadBlock_Managemnet(DevW25NxxObj_TypeDef *dev)
{
    uint8_t cmd[2] = {W25NXX_BB_MANAGEMENT};

    if ((dev == NULL) || \
        DevW25Nxx_Read(dev, cmd, sizeof(cmd)) != DevW25Nxx_Ok)
        return DevW25Nxx_Error;

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

static DevW25Nxx_Error_List DevW25Nxx_WriteReg_Set(DevW25NxxObj_TypeDef *dev, uint8_t reg_addr, uint8_t field_index, uint8_t val)
{
    uint8_t cmd[3] = {W25NXX_WRITE_STATUS_CMD, reg_addr};

    if ((dev == NULL) || \
        ((reg_addr != W25NXX_SR0_ADDR) && \
         (reg_addr != W25NXX_SR1_ADDR)))
        return DevW25Nxx_Error;

    if (!DevW25Nxx_Read(dev, cmd, sizeof(cmd)))
        return DevW25Nxx_Error;

    if (reg_addr == W25NXX_SR0_ADDR)
    {
        switch (field_index)
        {
            case BF_SRP_1: ConvertToSR0_RegFormat(&cmd[2])->bit.SRP_1 = val; break;
            case BF_WPE:   ConvertToSR0_RegFormat(&cmd[2])->bit.WPE   = val; break;
            case BF_TB:    ConvertToSR0_RegFormat(&cmd[2])->bit.TB    = val; break;
            case BF_BP_0:  ConvertToSR0_RegFormat(&cmd[2])->bit.BP_0  = val; break;
            case BF_BP_1:  ConvertToSR0_RegFormat(&cmd[2])->bit.BP_1  = val; break;
            case BF_BP_2:  ConvertToSR0_RegFormat(&cmd[2])->bit.BP_2  = val; break;
            case BF_BP_3:  ConvertToSR0_RegFormat(&cmd[2])->bit.BP_3  = val; break;
            case BF_SRP_0: ConvertToSR0_RegFormat(&cmd[2])->bit.SRP_0 = val; break;
            default: return DevW25Nxx_Error;
        }
    }
    else if (reg_addr == W25NXX_SR1_ADDR)
    {
        switch (field_index)
        {
            case BF_BUF:   ConvertToSR1_RegFormat(&cmd[2])->bit.BUF   = val; break;
            case BF_ECC_E: ConvertToSR1_RegFormat(&cmd[2])->bit.ECC_E = val; break;
            case BF_SR1_L: ConvertToSR1_RegFormat(&cmd[2])->bit.SR1_L = val; break;
            case BF_OTP_E: ConvertToSR1_RegFormat(&cmd[2])->bit.OTP_E = val; break;
            case BF_OTP_L: ConvertToSR1_RegFormat(&cmd[2])->bit.OTP_L = val; break;
            default: return DevW25Nxx_Error;
        }
    }
    
    if (!DevW25Nxx_Write(dev, cmd, sizeof(cmd)))
        return DevW25Nxx_Error;

    return DevW25Nxx_Ok;
}

static DevW25Nxx_Error_List DevW25Nxx_WriteEn(DevW25NxxObj_TypeDef *dev)
{
    uint8_t cmd[3];
    DevW25Nxx_SR2_TypeDef sr2;

    cmd[0] = W25NXX_WRITE_STATUS_CMD;
    cmd[1] = W25NXX_SR2_ADDR;
    cmd[2] = 0;

    if ((dev == NULL) || \
        (!DevW25Nxx_Read(dev, cmd, sizeof(cmd))))
        return DevW25Nxx_Error;

    sr2.val = cmd[2];
    sr2.bit.WEL = 0;
    cmd[2] = sr2.val;

    if (!DevW25Nxx_Write(dev, cmd, sizeof(cmd)))
        return DevW25Nxx_Error;

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
    if (DevW25Nxx_WriteEn(dev) == DevW25Nxx_Error)
        return DevW25Nxx_Error;

    

    return DevW25Nxx_Ok;
}

static DevW25Nxx_Error_List DevW25Nxx_Read_Page(DevW25NxxObj_TypeDef *dev, uint32_t addr, uint8_t *p_data, uint32_t size)
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

        /* get chip status */
        err = DevW25Nxx_Check_Read_Status(dev);
        if (err == DevW25Nxx_Error)
            return DevW25Nxx_Error;
        
        sys_time = dev->systick();
        while (err == DevW25Nxx_Busy)
        {
            if ((dev->systick() - sys_time) >= W25NXX_BUS_COMMU_TIMEOUT)
                return DevW25Nxx_TimeOut;

            err = DevW25Nxx_Check_Read_Status(dev);
            if (err == DevW25Nxx_Error)
                return DevW25Nxx_Error;
        }

        /* read page */
        // cmd[0] = W25NXX_PAGE_DATA_READ;
        // cmd[1] = ;
        // cmd[2] = ;
        // cmd[3] = ;

        dev->cs_ctl(false);
        state = dev->bus_tx(cmd, sizeof(cmd), W25NXX_BUS_COMMU_TIMEOUT);  /* send cmd */
        // state &= dev->bus_rx();
        dev->cs_ctl(true);

        if (!state)
            return DevW25Nxx_Error;
    }

    return DevW25Nxx_Error;
}
