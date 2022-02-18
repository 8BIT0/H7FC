#include "Dev_W25Qxx.h"
#include "Bsp_SPI.h"

static DevW25Qxx_Error_List DevW25Qxx_Init(DevW25QxxObj_TypeDef dev);
static DevW25Qxx_Error_List DevW25Qxx_Reset(DevW25QxxObj_TypeDef dev);
static DevW25Qxx_Error_List DevW25Qxx_Write(DevW25QxxObj_TypeDef dev, uint32_t WriteAddr, uint8_t *pData, uint32_t Size);
static DevW25Qxx_Error_List DevW25Qxx_Read(DevW25QxxObj_TypeDef dev, uint32_t ReadAddr, uint32_t *pData, uint32_t Size);
static DevW25Qxx_Error_List DevW25Qxx_EraseBlock(DevW25QxxObj_TypeDef dev, uint32_t Address);
static DevW25Qxx_Error_List DevW25Qxx_EraseChip(DevW25QxxObj_TypeDef dev);

/* without pin config in this down */
static BspSPI_NorModeConfig_TypeDef DevW25QxxSPI_Cfg = {
    .Mode = SPI_MODE_MASTER,
    .Direction = SPI_DIRECTION_2LINES,
    .DataSize = SPI_DATASIZE_8BIT,
    .CLKPolarity = SPI_POLARITY_HIGH,
    .CLKPhase = SPI_PHASE_2EDGE,
    .NSS = SPI_NSS_SOFT,
    .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128,
    .FirstBit = SPI_FIRSTBIT_MSB,
};

DevW25Qxx_TypeDef DevW25Q64 = {
    .init = DevW25Qxx_Init,
    .reset = DevW25Qxx_Reset,
    .write = DevW25Qxx_Write,
    .read = DevW25Qxx_Read,
    .erase_block = DevW25Qxx_EraseBlock,
    .erase_chip = DevW25Qxx_EraseChip,
};

/* DevW25Qxx Base SPI communicate interface */
static BspSpi_TypeDef *DevW25Qxx_GetSspiInstance(DevW25QxxObj_TypeDef dev)
{
    if ((dev.BusPort == NULL) || (dev.bus_type != DevW25Qxx_Norm_SpiBus))
        return NULL;

    return (BspSpi_TypeDef *)(dev.BusPort);
}

static bool DevW25Qxx_BusTrans(DevW25QxxObj_TypeDef dev, uint8_t *tx, uint16_t size)
{
    bool state = false;

    if ((DevW25Qxx_GetSspiInstance(dev) == NULL) || (DevW25Qxx_GetSspiInstance(dev)->trans == NULL))
        return false;

    dev.CSPin.ctl(true);

    state = DevW25Qxx_GetSspiInstance(dev)->trans(dev.bus_instance, tx, size, W25Qx_TIMEOUT_VALUE);

    dev.CSPin.ctl(false);

    return state;
}

static bool DevW25Qxx_BusReceive(DevW25QxxObj_TypeDef dev, uint8_t *rx, uint16_t size)
{
    bool state = false;

    if ((DevW25Qxx_GetSspiInstance(dev) == NULL) || (DevW25Qxx_GetSspiInstance(dev)->receive == NULL))
        return false;

    dev.CSPin.ctl(true);

    state = DevW25Qxx_GetSspiInstance(dev)->receive(dev.bus_instance, rx, size, W25Qx_TIMEOUT_VALUE);

    dev.CSPin.ctl(false);

    return state;
}

static bool DevW25Qxx_BusTrans_Receive(DevW25QxxObj_TypeDef dev, uint8_t *tx, uint8_t *rx, uint16_t size)
{
    bool state;

    if ((DevW25Qxx_GetSspiInstance(dev) == NULL) || (DevW25Qxx_GetSspiInstance(dev)->trans_receive == NULL))
        return false;

    dev.CSPin.ctl(true);

    state = DevW25Qxx_GetSspiInstance(dev)->trans_receive(dev.bus_instance, tx, rx, size, W25Qx_TIMEOUT_VALUE);

    dev.CSPin.ctl(false);

    return state;
}

/* W25Qxx device driver */
static bool DevW25Qxx_ReadID(DevW25QxxObj_TypeDef dev)
{
    uint8_t cmd[4] = {READ_ID_CMD, 0x00, 0x00, 0x00};
    uint8_t ID[2] = {0};

    /* Send the read ID command */
    DevW25Qxx_BusTrans(dev, cmd, sizeof(cmd));
    /* Reception of the data */
    DevW25Qxx_BusReceive(dev, ID, sizeof(ID));

    return true;
}

static DevW25Qxx_Error_List DevW25Qxx_Reset(DevW25QxxObj_TypeDef dev)
{
    uint8_t cmd[2] = {RESET_ENABLE_CMD, RESET_MEMORY_CMD};

    /* Send the reset command */
    if (DevW25Qxx_BusTrans(dev, cmd, sizeof(cmd)))
        return DevW25Qxx_Ok;

    return DevW25Qxx_Error;
}

static DevW25Qxx_Error_List DevW25Qxx_GetStatue(DevW25QxxObj_TypeDef dev)
{
    uint8_t cmd = READ_STATUS_REG1_CMD;
    bool trans_state = false;
    uint8_t dev_status;

    trans_state = DevW25Qxx_BusTrans_Receive(dev, &cmd, &dev_status, sizeof(dev_status));

    if (!trans_state)
        return DevW25Qxx_Error;

    /* Check the value of the register */
    if ((dev_status & W25Q128FV_FSR_BUSY) != 0)
        return DevW25Qxx_Busy;

    return DevW25Qxx_Ok;
}

static DevW25Qxx_Error_List DevW25Qxx_WriteEnable(DevW25QxxObj_TypeDef dev)
{
    uint8_t cmd = WRITE_ENABLE_CMD;
    uint32_t tickstart = dev.systick();
    bool trans_state = false;

    /* Send the read ID command */
    trans_state = DevW25Qxx_BusTrans(dev, &cmd, sizeof(cmd));

    if (!trans_state)
        return DevW25Qxx_Error;

    /* Wait the end of Flash writing */
    while (DevW25Qxx_GetStatue(dev) == DevW25Qxx_Busy)
    {
        /* Check for the Timeout */
        if ((dev.systick() - tickstart) > W25Qx_TIMEOUT_VALUE)
            return DevW25Qxx_TimeOut;
    }

    return DevW25Qxx_Ok;
}

static DevW25Qxx_Error_List DevW25Qxx_Init(DevW25QxxObj_TypeDef dev)
{
    if ((dev.CSPin.init == NULL) ||
        (dev.CSPin.ctl == NULL) ||
        (dev.BusPort.init == NULL) ||
        (dev.BusPort.trans == NULL) ||
        (dev.BusPort.receive == NULL) ||
        (dev.BusPort.trans_receive == NULL))
        return DevW25Qxx_Error;

    DevW25Qxx_GetSspiInstance(dev)->init();
    dev.CSPin.init();

    /* Reset W25Qxxx */
    if ((DevW25Qxx_Reset(dev) != DevW25Qxx_Ok) ||
        (DevW25Qxx_GetStatue(dev) != DevW25Qxx_Ok))
        return DevW25Qxx_Error;

    return DevW25Qxx_Ok;
}

static DevW25Qxx_Error_List DevW25Qxx_Read(DevW25QxxObj_TypeDef dev, uint32_t ReadAddr, uint32_t *pData, uint32_t Size)
{
    uint8_t cmd[4];

    /* Configure the command */
    cmd[0] = READ_CMD;
    cmd[1] = (uint8_t)(ReadAddr >> 16);
    cmd[2] = (uint8_t)(ReadAddr >> 8);
    cmd[3] = (uint8_t)(ReadAddr);

    if (!DevW25Qxx_BusTrans(dev, cmd, sizeof(cmd)) || !DevW25Qxx_BusReceive(dev, pData, Size))
        return DevW25Qxx_Error;

    return DevW25Qxx_Ok;
}

static DevW25Qxx_Error_List DevW25Qxx_Write(DevW25QxxObj_TypeDef dev, uint32_t WriteAddr, uint8_t *pData, uint32_t Size)
{
    uint8_t cmd[4];
    uint32_t end_addr, current_size, current_addr;
    uint32_t tickstart = dev.systick();

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
        if (DevW25Qxx_BusTrans(dev, cmd, sizeof(cmd)) || DevW25Qxx_BusTrans(dev, pData, current_size))
            return DevW25Qxx_Error;

        /* Wait the end of Flash writing */
        while (DevW25Qxx_GetStatue(dev) == DevW25Qxx_Busy)
        {
            /* Check for the Timeout */
            if ((dev.systick() - tickstart) > W25Qx_TIMEOUT_VALUE)
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

static DevW25Qxx_Error_List DevW25Qxx_EraseChip(DevW25QxxObj_TypeDef dev)
{
    uint8_t cmd = SECTOR_ERASE_CMD;
    uint32_t tickstart = dev.systick();

    if ((DevW25Qxx_WriteEnable(dev) != DevW25Qxx_Ok) ||
        (DevW25Qxx_BusTrans(dev, &cmd, sizeof(cmd)) != DevW25Qxx_Ok))
        return DevW25Qxx_Error;

    /* Wait the end of Flash writing */
    while (DevW25Qxx_GetStatue(dev) != DevW25Qxx_Busy)
    {
        /* Check for the Timeout */
        if ((dev.systick() - tickstart) > W25Q128FV_BULK_ERASE_MAX_TIME)
        {
            return DevW25Qxx_TimeOut;
        }
    }

    return DevW25Qxx_Ok;
}

static DevW25Qxx_Error_List DevW25Qxx_EraseBlock(DevW25QxxObj_TypeDef dev, uint32_t Address)
{
    uint8_t cmd[4];
    uint32_t tickstart = dev.systick();
    cmd[0] = SECTOR_ERASE_CMD;
    cmd[1] = (uint8_t)(Address >> 16);
    cmd[2] = (uint8_t)(Address >> 8);
    cmd[3] = (uint8_t)(Address);

    /* Enable write operations Send the read ID command */
    if ((DevW25Qxx_WriteEnable(dev) != DevW25Qxx_Ok) || (DevW25Qxx_BusTrans(dev, cmd, sizeof(cmd)) != DevW25Qxx_Ok))
        return DevW25Qxx_Error;

    /* Wait the end of Flash writing */
    while (DevW25Qxx_GetStatue(dev) == DevW25Qxx_Busy)
    {
        /* Check for the Timeout */
        if ((dev.systick() - tickstart) > W25Q128FV_SECTOR_ERASE_MAX_TIME)
        {
            return DevW25Qxx_TimeOut;
        }
    }

    return DevW25Qxx_Ok;
}
