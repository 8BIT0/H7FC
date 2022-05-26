#include "Bsp_SDIO.h"

static const GPIO_InitTypeDef BspSDIO_PinCfg = {
    .Mode = GPIO_MODE_AF_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
};

/* internal function */
static bool BspSDIO_PortCLK_Init(SDMMC_TypeDef *instance);
static void BspSDIO_PinCLK_Enable(GPIO_TypeDef *port);

/* external function */
static bool BspSDIO_Init(BspSDIO_Obj_TypeDef *obj);

BspSDIO_TypeDef BspSDIO = {
    .init = BspSDIO_Init,
    .read = NULL,
    .write = NULL,
};

static void BspSDIO_PinCLK_Enable(GPIO_TypeDef *port)
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

static bool BspSDIO_PortCLK_Init(SDMMC_TypeDef *instance)
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
    }

    return true;
}

static bool BspSDIO_Pin_Init(SD_TypeDef *type, BspSDIO_PinConfig_TypeDef *obj)
{
    GPIO_InitTypeDef GPIO_InitStruct = BspSDIO_PinCfg;

    if (obj == NULL)
        return false;

    if (type == SDMMC1)
    {
        BspSDIO_PinCLK_Enable(obj->CK_Port);
        BspSDIO_PinCLK_Enable(obj->CMD_Port);
        BspSDIO_PinCLK_Enable(obj->D0_Port);
        BspSDIO_PinCLK_Enable(obj->D1_Port);
        BspSDIO_PinCLK_Enable(obj->D2_Port);
        BspSDIO_PinCLK_Enable(obj->D3_Port);

        //     __HAL_RCC_GPIOC_CLK_ENABLE();
        //     /**SDMMC1 GPIO Configuration
        //     PD2     ------> SDMMC1_CMD
        //     PC11     ------> SDMMC1_D3
        //     PC10     ------> SDMMC1_D2
        //     PC12     ------> SDMMC1_CK
        //     PC9     ------> SDMMC1_D1
        //     PC8     ------> SDMMC1_D0
        //     */
        //     GPIO_InitStruct.Pin = GPIO_PIN_2;
        //     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        //     GPIO_InitStruct.Pull = GPIO_NOPULL;
        //     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        //     GPIO_InitStruct.Alternate = GPIO_AF12_SDIO1;
        //     HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        //     GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_10 | GPIO_PIN_12 | GPIO_PIN_9 | GPIO_PIN_8;
        //     GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        //     GPIO_InitStruct.Pull = GPIO_NOPULL;
        //     GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        //     GPIO_InitStruct.Alternate = GPIO_AF12_SDIO1;
        //     HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    }
}

static bool BspSDIO_Init(BspSDIO_Obj_TypeDef *obj)
{
    BspSDIO_PortCLK_Init(obj->instance);

    obj->hdl.Instance = obj->instance; // SDMMC1;
    obj->hdl.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
    obj->hdl.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    obj->hdl.Init.BusWide = SDMMC_BUS_WIDE_4B;
    obj->hdl.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    obj->hdl.Init.ClockDiv = 0;
    obj->hdl.Init.TranceiverPresent = SDMMC_TRANSCEIVER_NOT_PRESENT;

    if (HAL_SD_Init(&(obj->hdl)) != HAL_OK)
        return false;

    return true;
}
