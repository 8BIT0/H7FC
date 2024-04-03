#include "Dev_W25Qxx.h"

/* internal function */
static DevW25Qxx_ProdType_List DevW25Qxx_Get_ProdType(DevW25QxxObj_TypeDef *dev, uint16_t *id);

/* external function */
static DevW25Qxx_Error_List DevW25Qxx_Init(DevW25QxxObj_TypeDef *dev);
static DevW25Qxx_Error_List DevW25Qxx_Reset(DevW25QxxObj_TypeDef *dev);
static DevW25Qxx_Error_List DevW25Qxx_Write(DevW25QxxObj_TypeDef *dev, uint32_t WriteAddr, uint8_t *pData, uint32_t Size);
static DevW25Qxx_Error_List DevW25Qxx_Read(DevW25QxxObj_TypeDef *dev, uint32_t ReadAddr, uint8_t *pData, uint32_t Size);
static DevW25Qxx_Error_List DevW25Qxx_EraseSector(DevW25QxxObj_TypeDef *dev, uint32_t Address);
static DevW25Qxx_Error_List DevW25Qxx_EraseChip(DevW25QxxObj_TypeDef *dev);
static DevW25Qxx_DeviceInfo_TypeDef DevW25Qxx_Get_Info(DevW25QxxObj_TypeDef *dev);
static uint32_t DevW25Qxx_Get_Section_StartAddr(DevW25QxxObj_TypeDef *dev, uint32_t addr);

DevW25Qxx_TypeDef DevW25Qxx = {
    .init = DevW25Qxx_Init,
    .reset = DevW25Qxx_Reset,
    .write = DevW25Qxx_Write,
    .read = DevW25Qxx_Read,
    .erase_sector = DevW25Qxx_EraseSector,
    .erase_chip = DevW25Qxx_EraseChip,
    .info = DevW25Qxx_Get_Info,
    .get_section_start_addr = DevW25Qxx_Get_Section_StartAddr,
};

static bool DevW25Qxx_BusTrans(DevW25QxxObj_TypeDef *dev, uint8_t *tx, uint16_t size)
{
    if ((dev == NULL) || \
        (dev->bus_tx == NULL))
        return false;

    if (dev->bus_tx(tx, size, W25Qx_TIMEOUT_VALUE))
        return true;

    return false;
}

static bool DevW25Qxx_BusReceive(DevW25QxxObj_TypeDef *dev, uint8_t *rx, uint16_t size)
{
    if ((dev == NULL) || \
        (dev->bus_rx == NULL))
        return false;

    if (dev->bus_rx(rx, size, W25Qx_TIMEOUT_VALUE))
        return true;

    return false;
}

static bool DevW25Qxx_BusTrans_Receive(DevW25QxxObj_TypeDef *dev, uint8_t *tx, uint8_t *rx, uint16_t size)
{
    if ((dev == NULL) || \
        (dev->bus_trans == NULL))
        return false;

    if (dev->bus_trans(tx, rx, size, W25Qx_TIMEOUT_VALUE))
        return true;

    return false;
}

static DevW25Qxx_Error_List DevW25Qxx_Reset(DevW25QxxObj_TypeDef *dev)
{
    uint8_t cmd[2] = {RESET_ENABLE_CMD, RESET_MEMORY_CMD};
    bool state = false;

    if ((dev == NULL) || (dev->cs_ctl == NULL))
        return DevW25Qxx_Error;

    /* Send the reset command */
    dev->cs_ctl(false);
    state = DevW25Qxx_BusTrans(dev, cmd, sizeof(cmd));
    dev->cs_ctl(true);

    if (!state)
        return DevW25Qxx_Error;

    return DevW25Qxx_Ok;
}

static DevW25Qxx_Error_List DevW25Qxx_GetStatue(DevW25QxxObj_TypeDef *dev)
{
    uint8_t cmd = READ_STATUS_REG1_CMD;
    uint8_t dev_status = 0;
    bool state = false;

    if ((dev == NULL) || (dev->cs_ctl == NULL))
        return DevW25Qxx_Error;

    dev->cs_ctl(false);
    state = DevW25Qxx_BusTrans(dev, &cmd, sizeof(cmd)) & DevW25Qxx_BusReceive(dev, &dev_status, sizeof(dev_status));
    dev->cs_ctl(true);
    
    if (!state)
        return DevW25Qxx_Error; 

    /* Check the value of the register */
    if ((dev_status & W25Q128FV_FSR_BUSY) != 0)
        return DevW25Qxx_Busy;

    return DevW25Qxx_Ok;
}

static DevW25Qxx_Error_List DevW25Qxx_WriteEnable(DevW25QxxObj_TypeDef *dev)
{
    uint8_t cmd = WRITE_ENABLE_CMD;
    uint32_t tickstart = 0;
    bool trans_state = false;

    if ((dev == NULL) || (dev->cs_ctl == NULL) || (dev->systick == NULL))
        return DevW25Qxx_Error;

    /* Send the read ID command */
    dev->cs_ctl(false);
    trans_state = DevW25Qxx_BusTrans(dev, &cmd, sizeof(cmd));
    dev->cs_ctl(true);

    tickstart = dev->systick();

    if (!trans_state)
        return DevW25Qxx_Error;

    /* Wait the end of Flash writing */
    while (DevW25Qxx_GetStatue(dev) == DevW25Qxx_Busy)
    {
        /* Check for the Timeout */
        if ((dev->systick() - tickstart) > W25Qx_TIMEOUT_VALUE)
            return DevW25Qxx_TimeOut;
    }

    return DevW25Qxx_Ok;
}

static DevW25Qxx_Error_List DevW25Qxx_Init(DevW25QxxObj_TypeDef *dev)
{
    if ((dev == NULL) ||
        (dev->cs_ctl == NULL))
        return DevW25Qxx_Error;

    dev->init_state = DevW25Qxx_Error;
    
    /* Reset W25Qxxx */
    if (DevW25Qxx_Reset(dev) != DevW25Qxx_Ok)
        return DevW25Qxx_Error;

    if (DevW25Qxx_GetStatue(dev) != DevW25Qxx_Ok)
        return DevW25Qxx_Error;

    dev->prod_type = DevW25Qxx_Get_ProdType(dev, &dev->prod_code);

    if (dev->prod_type == DevW25Q_None)
        return DevW25Qxx_Error;

    dev->init_state = DevW25Qxx_Ok;
    return DevW25Qxx_Ok;
}

static DevW25Qxx_ProdType_List DevW25Qxx_Get_ProdType(DevW25QxxObj_TypeDef *dev, uint16_t *id)
{
    uint8_t ID_Req_CMD[] = {READ_ID_CMD, 0, 0, 0};
    uint8_t ID_Rx_buf[] = {0, 0};
    uint16_t ID = 0;

    if (dev && dev->cs_ctl)
    {
        dev->cs_ctl(false);
        DevW25Qxx_BusTrans(dev, ID_Req_CMD, sizeof(ID_Req_CMD));
        DevW25Qxx_BusReceive(dev, ID_Rx_buf, sizeof(ID_Rx_buf));
        dev->cs_ctl(true);
    
        ID |= ID_Rx_buf[0] << 8;
        ID |= ID_Rx_buf[1];
        *id = ID;

        switch(ID)
        {
            case W25Q08_DEV_ID:
                return DevW25Q_08;

            case W25Q16_DEV_ID:
                return DevW25Q_16;

            case W25Q32_DEV_ID:
                return DevW25Q_32;

            case W25Q64_DEV_ID:
                return DevW25Q_64;

            case W25Q128_DEV_ID:
                return DevW25Q_128;

            default:
                return DevW25Q_None;
        }
    }

    return DevW25Q_None;
}

static DevW25Qxx_Error_List DevW25Qxx_Read(DevW25QxxObj_TypeDef *dev, uint32_t ReadAddr, uint8_t *pData, uint32_t Size)
{
    uint8_t cmd[4];
    bool read_state = false;

    /* Configure the command */
    cmd[0] = READ_CMD;
    cmd[1] = (uint8_t)(ReadAddr >> 16);
    cmd[2] = (uint8_t)(ReadAddr >> 8);
    cmd[3] = (uint8_t)(ReadAddr);

    if ((dev == NULL) || (dev->cs_ctl == NULL) || (pData == NULL) || (Size == 0))
        return DevW25Qxx_Error; 

    dev->cs_ctl(false);
    read_state = DevW25Qxx_BusTrans(dev, cmd, sizeof(cmd)) & DevW25Qxx_BusReceive(dev, pData, Size);
    dev->cs_ctl(true);

    if (read_state)
        return DevW25Qxx_Ok;

    return DevW25Qxx_Error; 
}

static DevW25Qxx_Error_List DevW25Qxx_Write(DevW25QxxObj_TypeDef *dev, uint32_t WriteAddr, uint8_t *pData, uint32_t Size)
{
    uint8_t cmd[4];
    uint32_t end_addr, current_size, current_addr;
    uint32_t tickstart = 0;

    if ((dev == NULL) || (dev->cs_ctl == NULL) || (dev->systick == NULL) || (pData == NULL) || (Size == 0))
        return DevW25Qxx_Error;

    /* Calculation of the size between the write address and the end of the page */
    current_addr = 0;

    while (current_addr <= WriteAddr)
    {
        current_addr += W25Q128FV_PAGE_SIZE;
    }
    current_size = current_addr - WriteAddr;

    /* Check if the size of the data is less than the remaining place in the page */
    if (current_size > Size)
    {
        current_size = Size;
    }

    /* Initialize the adress variables */
    current_addr = WriteAddr;
    end_addr = WriteAddr + Size;

    tickstart = dev->systick();

    /* Perform the write page by page */
    do
    {
        /* Configure the command */
        cmd[0] = PAGE_PROG_CMD;
        cmd[1] = (uint8_t)(current_addr >> 16);
        cmd[2] = (uint8_t)(current_addr >> 8);
        cmd[3] = (uint8_t)(current_addr);

        /* Enable write operations */
        if (DevW25Qxx_WriteEnable(dev) != DevW25Qxx_Ok)
            return DevW25Qxx_Error;

        /* Send the command Transmission of the data */
        dev->cs_ctl(false);
        DevW25Qxx_BusTrans(dev, cmd, sizeof(cmd));
        DevW25Qxx_BusTrans(dev, pData, current_size);
        dev->cs_ctl(true);

        /* Wait the end of Flash writing */
        while (DevW25Qxx_GetStatue(dev) == DevW25Qxx_Busy)
        {
            /* Check for the Timeout */
            if ((dev->systick() - tickstart) > W25Qx_TIMEOUT_VALUE)
            {
                return DevW25Qxx_TimeOut;
            }
        }

        /* Update the address and size variables for next page programming */
        current_addr += current_size;
        pData += current_size;
        current_size = ((current_addr + W25Q128FV_PAGE_SIZE) > end_addr) ? (end_addr - current_addr) : W25Q128FV_PAGE_SIZE;
    } while (current_addr < end_addr);

    return DevW25Qxx_Ok;
}

static DevW25Qxx_Error_List DevW25Qxx_EraseChip(DevW25QxxObj_TypeDef *dev)
{
    uint8_t cmd = CHIP_ERASE_CMD;
    uint32_t tickstart = 0;
    bool erase_state = false;

    if ((dev == NULL) || (dev->cs_ctl == NULL) || (dev->systick == NULL))
        return DevW25Qxx_Error;

    tickstart = dev->systick();

    if (DevW25Qxx_WriteEnable(dev) != DevW25Qxx_Ok)
        return DevW25Qxx_Error;

    dev->cs_ctl(false);
    erase_state = DevW25Qxx_BusTrans(dev, &cmd, sizeof(cmd));
    dev->cs_ctl(true);

    if (!erase_state)
        return DevW25Qxx_Error;

    /* Wait the end of Flash writing */
    while (DevW25Qxx_GetStatue(dev) != DevW25Qxx_Busy)
    {
        /* Check for the Timeout */
        if ((dev->systick() - tickstart) > W25Q128FV_BULK_ERASE_MAX_TIME)
        {
            return DevW25Qxx_TimeOut;
        }
    }

    return DevW25Qxx_Ok;
}

static DevW25Qxx_Error_List DevW25Qxx_EraseSector(DevW25QxxObj_TypeDef *dev, uint32_t Address)
{
    uint8_t cmd[4];
    uint32_t tickstart = 0;
    bool erase_state = false;
    cmd[0] = SECTOR_ERASE_CMD;
    cmd[1] = (uint8_t)(Address >> 16);
    cmd[2] = (uint8_t)(Address >> 8);
    cmd[3] = (uint8_t)(Address);

    if ((dev == NULL) || (dev->cs_ctl == NULL) || (dev->systick == NULL))
        return DevW25Qxx_Error;

    tickstart = dev->systick();

    /* Enable write operations Send the read ID command */
    if (DevW25Qxx_WriteEnable(dev) != DevW25Qxx_Ok)
        return DevW25Qxx_Error;

    dev->cs_ctl(false);
    erase_state = DevW25Qxx_BusTrans(dev, cmd, sizeof(cmd));
    dev->cs_ctl(true);

    if (!erase_state)
        return DevW25Qxx_Error;

    /* Wait the end of Flash writing */
    while (DevW25Qxx_GetStatue(dev) == DevW25Qxx_Busy)
    {
        /* Check for the Timeout */
        if ((dev->systick() - tickstart) > W25Q128FV_SECTOR_ERASE_MAX_TIME)
        {
            return DevW25Qxx_TimeOut;
        }
    }

    return DevW25Qxx_Ok;
}

static DevW25Qxx_DeviceInfo_TypeDef DevW25Qxx_Get_Info(DevW25QxxObj_TypeDef *dev)
{
    DevW25Qxx_DeviceInfo_TypeDef info;
    
    memset(&info, 0, sizeof(DevW25Qxx_DeviceInfo_TypeDef));

    if (dev && (dev->prod_type != DevW25Q_None))
    {
        info.start_addr = W25QXX_BASE_ADDRESS;
        info.prod_type = dev->prod_type;
        info.prod_code = dev->prod_code;

        /* currently we only have such type of flash chip on the evk or fc-board */
        switch ((uint8_t)(dev->prod_type))
        {
            case DevW25Q_64:
                info.flash_size     = W25Q64FV_FLASH_SIZE;

                info.page_num       = W25Q64FV_PAGE_NUM;
                info.page_size      = W25Q64FV_PAGE_SIZE;

                info.sector_num     = W25Q64FV_SECTOR_NUM;
                info.sector_size    = W25Q64FV_SECTOR_SIZE;

                info.subsector_num  = W25Q64FV_SUBSECTOR_NUM;
                info.subsector_size = W25Q64FV_SUBSECTOR_SIZE;
                break;

            case DevW25Q_128:
                info.flash_size     = W25Q128FV_FLASH_SIZE;
                
                info.page_num       = W25Q128FV_PAGE_NUM;
                info.page_size      = W25Q128FV_PAGE_SIZE;

                info.sector_num     = W25Q128FV_SECTOR_NUM;
                info.sector_size    = W25Q128FV_SECTOR_SIZE;

                info.subsector_num  = W25Q128FV_SUBSECTOR_NUM;
                info.subsector_size = W25Q128FV_SUBSECTOR_SIZE;
                break;

            default:
                break;
        }
    }

    return info;
}

static uint32_t DevW25Qxx_Get_Section_StartAddr(DevW25QxxObj_TypeDef *dev, uint32_t addr)
{
    if (dev && (dev->init_state == DevW25Qxx_Ok))
    {
        return (addr / W25Q64FV_SUBSECTOR_SIZE) * W25Q64FV_SUBSECTOR_SIZE;
    }

    return 0;
}
