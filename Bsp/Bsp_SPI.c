#include "Bsp_SPI.h"

/* internal function */
static bool BspSPI_NormalMode_Init(BspSPI_NorModeConfig_TypeDef spi_cfg, SPI_HandleTypeDef *spi_instance);
static bool BspSPI_DeInit(BspSPI_NorModeConfig_TypeDef spi_cfg);
static bool BspSPI_Trans(SPI_HandleTypeDef *spi_instance, uint8_t *tx, uint16_t size, uint16_t time_out);
static bool BspSPI_Receive(SPI_HandleTypeDef *spi_instance, uint8_t *rx, uint16_t size, uint16_t time_out);
static uint16_t BspSPI_TransReceive(SPI_HandleTypeDef *spi_instance, uint8_t *tx, uint8_t *rx, uint16_t size, uint16_t time_out);
static bool BspSPI_Set_CLKSpeed(SPI_HandleTypeDef *spi_instance, uint32_t speed);

/* SPI0 Object */
BspSpi_TypeDef BspSPI = {
    .init = BspSPI_NormalMode_Init,
    .deinit = BspSPI_DeInit,
    .trans = BspSPI_Trans,
    .receive = BspSPI_Receive,
    .trans_receive = BspSPI_TransReceive,
    .set_speed = BspSPI_Set_CLKSpeed,
};

static const GPIO_InitTypeDef BspSPI_Pin_Cfg = {
    .Alternate = 0,
    .Mode = GPIO_MODE_AF_PP,
    .Pin = 0,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
};

static bool BspSPI_PinInit(BspSPI_PinConfig_TypeDef pin_cfg)
{
    GPIO_InitTypeDef GPIO_InitStruct = BspSPI_Pin_Cfg;

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /* mosi pin init */
    GPIO_InitStruct.Pin = pin_cfg.pin_mosi;
    GPIO_InitStruct.Alternate = pin_cfg.pin_Alternate;
    HAL_GPIO_Init(pin_cfg.port_mosi, &GPIO_InitStruct);

    /* miso pin init */
    GPIO_InitStruct.Pin = pin_cfg.pin_miso;
    GPIO_InitStruct.Alternate = pin_cfg.pin_Alternate;
    HAL_GPIO_Init(pin_cfg.port_miso, &GPIO_InitStruct);

    /* clk pin init */
    GPIO_InitStruct.Pin = pin_cfg.pin_clk;
    GPIO_InitStruct.Alternate = pin_cfg.pin_Alternate;
    HAL_GPIO_Init(pin_cfg.port_clk, &GPIO_InitStruct);

    return true;
}

static bool BspSPI_NormalMode_Init(BspSPI_NorModeConfig_TypeDef spi_cfg, SPI_HandleTypeDef *spi_instance)
{
    SPI_HandleTypeDef SPI_InitStructure;

    if (spi_cfg.Instance == SPI1)
    {
        __SPI1_CLK_ENABLE();
    }
    else if (spi_cfg.Instance == SPI2)
    {
        __SPI2_CLK_ENABLE();
    }
    else if (spi_cfg.Instance == SPI3)
    {
        __SPI3_CLK_ENABLE();
    }
    else if (spi_cfg.Instance == SPI4)
    {
        __SPI4_CLK_ENABLE();
    }
    else if (spi_cfg.Instance == SPI5)
    {
        __SPI5_CLK_ENABLE();
    }
    else if (spi_cfg.Instance == SPI6)
    {
        __SPI6_CLK_ENABLE();
    }
    else
        return false;

    BspSPI_PinInit(spi_cfg.Pin);

    SPI_InitStructure.Instance = spi_cfg.Instance;                        // SPI2;
    SPI_InitStructure.Init.Mode = SPI_MODE_MASTER;                        // SPI_MODE_MASTER;
    SPI_InitStructure.Init.Direction = SPI_DIRECTION_2LINES;              // SPI_DIRECTION_2LINES;
    SPI_InitStructure.Init.DataSize = SPI_DATASIZE_8BIT;                  // SPI_DATASIZE_8BIT;
    SPI_InitStructure.Init.CLKPolarity = spi_cfg.CLKPolarity;             // SPI_POLARITY_LOW;
    SPI_InitStructure.Init.CLKPhase = spi_cfg.CLKPhase;                   // SPI_PHASE_1EDGE;
    SPI_InitStructure.Init.NSS = SPI_NSS_SOFT;                            // SPI_NSS_SOFT;
    SPI_InitStructure.Init.BaudRatePrescaler = spi_cfg.BaudRatePrescaler; // SPI_BAUDRATEPRESCALER_2;
    SPI_InitStructure.Init.FirstBit = SPI_FIRSTBIT_MSB;                   // SPI_FIRSTBIT_MSB;
    SPI_InitStructure.Init.TIMode = SPI_TIMODE_DISABLE;
    SPI_InitStructure.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    SPI_InitStructure.Init.CRCPolynomial = 0;
    SPI_InitStructure.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    SPI_InitStructure.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    SPI_InitStructure.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    SPI_InitStructure.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    SPI_InitStructure.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    SPI_InitStructure.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    SPI_InitStructure.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    SPI_InitStructure.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    SPI_InitStructure.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    SPI_InitStructure.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    SPI_InitStructure.Init.IOSwap = SPI_IO_SWAP_DISABLE;

    if (HAL_SPI_Init(&SPI_InitStructure) != HAL_OK)
        return false;

    memcpy(spi_instance, &SPI_InitStructure, sizeof(SPI_InitStructure));

    return true;
}

static bool BspSPI_Set_CLKSpeed(SPI_HandleTypeDef *spi_instance, uint32_t speed)
{
    if (spi_instance == NULL)
        return false;

    spi_instance->Init.BaudRatePrescaler = speed;

    return true;
}

static bool BspSPI_DeInit(BspSPI_NorModeConfig_TypeDef spi_cfg)
{
    if (spi_cfg.Instance == SPI1)
    {
        __SPI1_CLK_DISABLE();
    }
    else if (spi_cfg.Instance == SPI2)
    {
        __SPI2_CLK_DISABLE();
    }
    else if (spi_cfg.Instance == SPI3)
    {
        __SPI3_CLK_DISABLE();
    }
    else if (spi_cfg.Instance == SPI4)
    {
        __SPI4_CLK_DISABLE();
    }
    else if (spi_cfg.Instance == SPI5)
    {
        __SPI5_CLK_DISABLE();
    }
    else if (spi_cfg.Instance == SPI6)
    {
        __SPI6_CLK_DISABLE();
    }
    else
        return false;

    HAL_GPIO_DeInit(spi_cfg.Pin.port_mosi, spi_cfg.Pin.pin_mosi);
    HAL_GPIO_DeInit(spi_cfg.Pin.port_miso, spi_cfg.Pin.pin_miso);
    HAL_GPIO_DeInit(spi_cfg.Pin.port_clk, spi_cfg.Pin.pin_clk);

    return true;
}

static bool BspSPI_Trans(SPI_HandleTypeDef *spi_instance, uint8_t *tx, uint16_t size, uint16_t time_out)
{
    if (HAL_SPI_Transmit(spi_instance, tx, size, time_out) == HAL_OK)
        return true;

    return false;
}

static bool BspSPI_Receive(SPI_HandleTypeDef *spi_instance, uint8_t *rx, uint16_t size, uint16_t time_out)
{
    if (HAL_SPI_Receive(spi_instance, rx, size, time_out) == HAL_OK)
        return true;

    return false;
}

static uint16_t BspSPI_TransReceive(SPI_HandleTypeDef *spi_instance, uint8_t *tx, uint8_t *rx, uint16_t size, uint16_t time_out)
{
    if (HAL_SPI_TransmitReceive(spi_instance, tx, rx, size, time_out) == HAL_OK)
        return true;

    return false;
}
