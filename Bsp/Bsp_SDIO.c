#include "Bsp_SDIO.h"

/* external function */
static bool BspSDIO_Init(void);
static SD_HandleTypeDef SD_handler;

BspSDIO_TypeDef BspSDIO = {
    .init = BspSDIO_Init,
    .read = NULL,
    .write = NULL,
};

static bool BspSDIO_Pin_Init()
{
    // GPIO_InitTypeDef GPIO_InitStruct = {0};
    // RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    // if (hsd->Instance == SDMMC1)
    // {
    //     /* USER CODE BEGIN SDMMC1_MspInit 0 */

    //     /* USER CODE END SDMMC1_MspInit 0 */
    //     /** Initializes the peripherals clock
    //      */
    //     PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDMMC;
    //     PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL;
    //     if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    //     {
    //         Error_Handler();
    //     }

    //     /* Peripheral clock enable */
    //     __HAL_RCC_SDMMC1_CLK_ENABLE();

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
    // }
}

static bool BspSDIO_Init(void)
{
    SD_handler.Instance = SDMMC1;
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
