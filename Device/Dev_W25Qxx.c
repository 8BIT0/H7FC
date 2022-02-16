#include "Dev_W25Qxx.h"
#include "Bsp_SPI.h"

/* DevW25Qxx Base SPI communicate interface */
static bool DevW25Qxx_BusTrans(DevW25QxxObj_TypeDef dev, uint8_t *tx, uint16_t size)
{
    if (dev.trans == NULL)
        return false;

    return dev.trans(dev.bus_instance, tx, size, W25Qx_TIMEOUT_VALUE);
}

static bool DevW25Qxx_BusReceive(DevW25QxxObj_TypeDef dev, uint8_t *rx, uint16_t size)
{
    if (dev.receive == NULL)
        return false;

    return dev.receive(dev.bus_instance, rx, size, W25Qx_TIMEOUT_VALUE);
}

static bool DevW25Qxx_BusTrans_Receive(DevW25QxxObj_TypeDef dev, uint8_t *tx, uint8_t *rx, uint16_t size)
{
    if (dev.trans_receive == NULL)
        return false;

    return dev.trans_receive(dev.bus_instance, tx, rx, size, W25Qx_TIMEOUT_VALUE);
}

/* W25Qxx device driver */
static DevW25Qxx_Error_List DevW25Qxx_Read(DevW25QxxObj_TypeDef dev, uint32_t addr, uint32_t *rx, uint32_t size)
{
    dev.cs_ctl(true);

    dev.cs_ctl(false);

    return DevW25Qxx_Ok;
}

static DevW25Qxx_Error_List DevW25Qxx_Write(DevW25QxxObj_TypeDef dev, uint32_t addr, uint8_t *tx, uint32_t size)
{
    dev.cs_ctl(true);

    dev.cs_ctl(false);

    return DevW25Qxx_Ok;
}

static DevW25Qxx_Error_List DevW25Qxx_EraseChip()
{
}

static DevW25Qxx_Error_List DevW25Qxx_EraseBlock(DevW25QxxObj_TypeDef dev, DevW25Qxx_EraseType_List type, uint32_t addr, uint32_t size)
{
    dev.cs_ctl(true);

    dev.cs_ctl(false);

    return DevW25Qxx_Ok;
}

static bool DevW25Qxx_ReadID(DevW25QxxObj_TypeDef dev)
{
    if ((dev == NULL) || (dev->cs_ctl == NULL))
        return false;

    dev.cs_ctl(true);

    dev.cs_ctl(false);

    return true;
}

static bool DevW25Qxx_Reset(DevW25QxxObj_TypeDef dev)
{
    bool state = false;
    uint8_t cmd[2] = {RESET_ENABLE_CMD, RESET_MEMORY_CMD};

    dev.cs_ctl(true);

    /* Send the reset command */
    state = DevW25Qxx_BusTrans(dev, cmd, sizeof(cmd));

    dev.cs_ctl(false);

    return state;
}

static uint8_t DevW25Qxx_GetStatue()
{
    uint8_t cmd = READ_STATUS_REG1_CMD;
    bool trans_state = false;
    uint8_t dev_status;

    dev.cs_ctl(true);

    /* Send the read status command */
    HAL_SPI_Transmit(&hspi1, cmd, 1, W25Qx_TIMEOUT_VALUE);
    /* Reception of the data */
    HAL_SPI_Receive(&hspi1, &status, 1, W25Qx_TIMEOUT_VALUE);

    dev.cs_ctl(false);

    /* Check the value of the register */
    if ((status & W25Q128FV_FSR_BUSY) != 0)
    {
        return W25Qx_BUSY;
    }
    else
    {
        return W25Qx_OK;
    }
}

static bool DevW25Qxx_WriteEnable()
{
    uint8_t cmd[] = {WRITE_ENABLE_CMD};
    uint32_t tickstart = HAL_GetTick();

    /*Select the FLASH: Chip Select low */
    W25Qx_Enable();
    /* Send the read ID command */
    HAL_SPI_Transmit(&hspi1, cmd, 1, W25Qx_TIMEOUT_VALUE);
    /*Deselect the FLASH: Chip Select high */
    W25Qx_Disable();

    /* Wait the end of Flash writing */
    while (DevW25Qxx_GetStatue() == W25Qx_BUSY)
        ;
    {
        /* Check for the Timeout */
        if ((HAL_GetTick() - tickstart) > W25Qx_TIMEOUT_VALUE)
        {
            return W25Qx_TIMEOUT;
        }
    }

    return W25Qx_OK;
}

static bool DevW25Qxx_Init(DevW25QxxObj_TypeDef *dev)
{
    if ((dev == NULL) ||
        (dev->cs_init == NULL) ||
        (dev->cs_ctl == NULL) ||
        (dev->bus_init == NULL) ||
        (dev->bus_trans_buff == NULL))
        return false;

    dev->bus_init();
    dev->cs_init();

    return true;
}
