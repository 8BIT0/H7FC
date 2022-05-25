#include "Bsp_SDIO.h"

static const GPIO_InitTypeDef BspSDIO_PinCfg = {
    .Mode = GPIO_MODE_AF_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
};

/* internal function */
static bool BspSDIO_CLK_Init(SDMMC_TypeDef *instance);

/* external function */
static bool BspSDIO_Init(void);
static SD_HandleTypeDef SD_handler;

BspSDIO_TypeDef BspSDIO = {
    .init = BspSDIO_Init,
    .read = NULL,
    .write = NULL,
};

static bool BspSDIO_CLK_Init(SDMMC_TypeDef *instance)
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

static bool BspSDIO_Pin_Init(BspSDIO_Obj_TypeDef *obj)
{
    GPIO_InitTypeDef GPIO_InitStruct = BspSDIO_PinCfg;

    if (obj->instance == SDMMC1)
    {
        //     __HAL_RCC_GPIOD_CLK_ENABLE();
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
    BspSDIO_CLK_Init(obj->instance);

    SD_handler.Instance = obj->instance; // SDMMC1;
    SD_handler.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
    SD_handler.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    SD_handler.Init.BusWide = SDMMC_BUS_WIDE_4B;
    SD_handler.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    SD_handler.Init.ClockDiv = 0;
    SD_handler.Init.TranceiverPresent = SDMMC_TRANSCEIVER_NOT_PRESENT;

    if (HAL_SD_Init(&SD_handler) != HAL_OK)
        return false;

    return true;
}
