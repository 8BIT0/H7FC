#include "Bsp_QSPI.h"
#include "Bsp_GPIO.h"
#include "stm32h7xx_hal.h"

/* external function */
static bool Bsp_QSPI_Init(BspQSPI_Config_TypeDef *obj);
static bool Bsp_QSPI_Command(BspQSPI_Config_TypeDef *obj, uint32_t mode, uint32_t dummy_cyc, uint32_t nb_data, uint32_t cmd);

BspQSpi_TypeDef BspQspi = {
    .init = Bsp_QSPI_Init,
    .cmd  = Bsp_QSPI_Command,
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

    __HAL_RCC_QSPI_CLK_ENABLE();

    /* pin init */


    obj->p_qspi.Instance                = QUADSPI;
	HAL_QSPI_DeInit(obj->p_qspi);
	obj->p_qspi.Init.ClockPrescaler     = 1;
	obj->p_qspi.Init.FifoThreshold      = 32;
	obj->p_qspi.Init.SampleShifting     = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
	obj->p_qspi.Init.FlashSize          = 22;
	obj->p_qspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
	obj->p_qspi.Init.ClockMode          = QSPI_CLOCK_MODE_3;
	obj->p_qspi.Init.FlashID            = QSPI_FLASH_ID_1;
	obj->p_qspi.Init.DualFlash          = QSPI_DUALFLASH_DISABLE;
	HAL_QSPI_Init(obj->p_qspi);

    obj->init_state = true;
    return true;
}

static bool Bsp_QSPI_Trans(BspQSPI_Config_TypeDef *obj, uint32_t addr, uint32_t cmd, uint8_t *p_data, uint32_t len)
{
    if ((obj == NULL) || \
        (obj->p_qspi == NULL) || \
        !obj->init_state)
        return false;

    return true;
}

static bool Bsp_QSPI_Recv(BspQSPI_Config_TypeDef *obj, uint32_t addr, uint32_t cmd, uint8_t *p_data, uint32_t len)
{
    if ((obj == NULL) || \
        (obj->p_qspi == NULL) || \
        !obj->init_state)
        return false;

    return true;
}

static bool Bsp_QSPI_Command(BspQSPI_Config_TypeDef *obj, uint32_t mode, uint32_t dummy_cyc, uint32_t nb_data, uint32_t cmd)
{
    QSPI_CommandTypeDef s_command;

    memset(&s_command, 0, sizeof(QSPI_CommandTypeDef));

	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
	s_command.AddressMode       = QSPI_ADDRESS_NONE;
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
	s_command.DataMode          = mode;
    s_command.NbData            = nb_data;
	s_command.DummyCycles       = dummy_cyc;
	s_command.Instruction       = cmd;

    if ((obj == NULL) || \
        (obj->p_qspi == NULL) || \
        !obj->init_state)
        return false;

    if (HAL_QSPI_Command(obj->p_qspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        return false;

    return true;
}

static bool Bsp_QSPI_Polling(BspQSPI_Config_TypeDef *obj)
{
	QSPI_CommandTypeDef s_command;
	QSPI_AutoPollingTypeDef s_config;

    memset(&s_command, 0, sizeof(QSPI_CommandTypeDef));
    memset(&s_config, 0, sizeof(QSPI_AutoPollingTypeDef));

    if ((obj == NULL) || \
        (obj->p_qspi == NULL) || \
        !obj->init_state)
        return false;

    if (HAL_QSPI_AutoPolling(obj->p_qspi, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        return false;

    return true;
}
