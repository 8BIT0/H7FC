#include "Bsp_SDMMC.h"

static const GPIO_InitTypeDef BspSDMMC_PinCfg = {
    .Mode = GPIO_MODE_AF_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
};

/* internal function */
static bool BspSDMMC_PortCLK_Init(SDMMC_TypeDef *instance);
static void BspSDMMC_PinCLK_Enable(GPIO_TypeDef *port);

/* external function */
static bool BspSDMMC_Init(BspSDIO_Obj_TypeDef *obj);

BspSDIO_TypeDef BspSDIO = {
    .init = BspSDMMC_Init,
    .read = NULL,
    .write = NULL,
};

static void BspSDMMC_PinCLK_Enable(GPIO_TypeDef *port)
{
    if (port == GPIOA)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    else if (port == GPIOB)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
    else if (port == GPIOC)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
    else if (port == GPIOD)
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }
    else if (port == GPIOE)
    {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    }
    else if (port == GPIOF)
    {
        __HAL_RCC_GPIOF_CLK_ENABLE();
    }
    else if (port == GPIOG)
    {
        __HAL_RCC_GPIOG_CLK_ENABLE();
    }
    else if (port == GPIOH)
    {
        __HAL_RCC_GPIOH_CLK_ENABLE();
    }
    else if (port == GPIOJ)
    {
        __HAL_RCC_GPIOJ_CLK_ENABLE();
    }
    else if (port == GPIOK)
    {
        __HAL_RCC_GPIOK_CLK_ENABLE();
    }
}

static bool BspSDMMC_PortCLK_Init(SDMMC_TypeDef *instance)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    if (instance == NULL)
        return false;

    if (instance == SDMMC1)
    {
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDMMC;
        PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
            return false;

        __HAL_RCC_SDMMC1_CLK_ENABLE();
        return true;
    }

    return true;
}

static bool BspSDMMC_Pin_Init(SD_TypeDef *type, BspSDIO_PinConfig_TypeDef *obj)
{
    GPIO_InitTypeDef GPIO_InitStruct = BspSDMMC_PinCfg;

    if (obj == NULL)
        return false;

    if (type == SDMMC1)
    {
        GPIO_InitStruct.Alternate = obj->Alternate;

        BspSDMMC_PinCLK_Enable(obj->CK_Port);
        GPIO_InitStruct.Pin = obj->CK_Pin;
        HAL_GPIO_Init(obj->CK_Port, &GPIO_InitStruct);

        BspSDMMC_PinCLK_Enable(obj->CMD_Port);
        GPIO_InitStruct.Pin = obj->CMD_Pin;
        HAL_GPIO_Init(obj->CMD_Port, &GPIO_InitStruct);

        BspSDMMC_PinCLK_Enable(obj->D0_Port);
        GPIO_InitStruct.Pin = obj->D0_Pin;
        HAL_GPIO_Init(obj->D0_Port, &GPIO_InitStruct);

        BspSDMMC_PinCLK_Enable(obj->D1_Port);
        GPIO_InitStruct.Pin = obj->D1_Pin;
        HAL_GPIO_Init(obj->D1_Port, &GPIO_InitStruct);

        BspSDMMC_PinCLK_Enable(obj->D2_Port);
        GPIO_InitStruct.Pin = obj->D2_Pin;
        HAL_GPIO_Init(obj->D2_Port, &GPIO_InitStruct);

        BspSDMMC_PinCLK_Enable(obj->D3_Port);
        GPIO_InitStruct.Pin = obj->D3_Pin;
        HAL_GPIO_Init(obj->D3_Port, &GPIO_InitStruct);

        return true;
    }

    return false;
}

static bool BspSDMMC_Init(BspSDIO_Obj_TypeDef *obj)
{
    BspSDMMC_PortCLK_Init(obj->instance);
    BspSDMMC_Pin_Init(obj->instance, obj->pin);

    obj->hdl->Instance = obj->instance; // SDMMC1;
    obj->hdl->Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
    obj->hdl->Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    obj->hdl->Init.BusWide = SDMMC_BUS_WIDE_4B;
    obj->hdl->Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    obj->hdl->Init.ClockDiv = 0;
    obj->hdl->Init.TranceiverPresent = SDMMC_TRANSCEIVER_NOT_PRESENT;

    if (HAL_SD_Init(&(obj->hdl)) != HAL_OK)
        return false;

    return true;
}

static bool BspSDMMC_Read(BspSDIO_Obj_TypeDef *obj, uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks)
{
    bool sd_state = true;

    if (HAL_SD_ReadBlocks(obj->hdl, (uint8_t *)pData, ReadAddr, NumOfBlocks, SDMMC_DATATIMEOUT) != HAL_OK)
        sd_state = false;

    return sd_state;
}

static bool BspSDMMC_Write(BspSDIO_Obj_TypeDef *obj, uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks)
{
    bool sd_state = true;

    if (HAL_SD_WriteBlocks(obj->hdl, (uint8_t *)pData, WriteAddr, NumOfBlocks, SDMMC_DATATIMEOUT) != HAL_OK)
        sd_state = false;

    return sd_state;
}

static bool BspSDMMC_Erase(BspSDIO_Obj_TypeDef *obj, uint32_t StartAddr, uint32_t EndAddr)
{
    bool sd_state = true;

    if (HAL_SD_Erase(obj->hdl, StartAddr, EndAddr) != HAL_OK)
        sd_state = false;

    return sd_state;
}
