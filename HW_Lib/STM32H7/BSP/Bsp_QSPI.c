#include "Bsp_QSPI.h"
#include "Bsp_GPIO.h"
#include "stm32h7xx_hal.h"

/* external function */
static bool Bsp_QSPI_Init(BspQSPI_Config_TypeDef *obj);

BspQSpi_TypeDef BspQspi = {
    .init = Bsp_QSPI_Init,
};

static bool Bsp_QSPI_Init(BspQSPI_Config_TypeDef *obj)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    if ((obj == NULL) || \
        (obj->p_qspi == NULL))
        return false;
    
    obj->init_state = false;

    /* clock init */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_QSPI;
    PeriphClkInitStruct.QspiClockSelection = RCC_QSPICLKSOURCE_D1HCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
        return false;

    /* pin init */


    obj->p_qspi.Instance                    = QUADSPI;
	HAL_QSPI_DeInit(obj->p_qspi);
	obj->p_qspi.Init.ClockPrescaler         = 1;
	obj->p_qspi.Init.FifoThreshold          = 32;
	obj->p_qspi.Init.SampleShifting         = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
	obj->p_qspi.Init.FlashSize              = 22;
	obj->p_qspi.Init.ChipSelectHighTime     = QSPI_CS_HIGH_TIME_1_CYCLE;
	obj->p_qspi.Init.ClockMode              = QSPI_CLOCK_MODE_3;
	obj->p_qspi.Init.FlashID                = QSPI_FLASH_ID_1;
	obj->p_qspi.Init.DualFlash              = QSPI_DUALFLASH_DISABLE;
	HAL_QSPI_Init(obj->p_qspi);

    obj->init_state = true;
    return true;
}

