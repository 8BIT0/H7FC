#include "Bsp_SPI.h"
#include "stm32h7xx.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_spi.h"

/* internal function */
static bool BspSPI_Init();
static void BspSPI_TransByte(uint8_t tx);
static uint8_t BspSPI_ReceiveByte(void);
static uint8_t BspSPI_TransMitByte(uint8_t tx);
static uint16_t BspSPI_TransMitBuff(uint8_t *tx, uint8_t *rx, uint16_t size);

/* SPI0 Object */
BspSpi_TypeDef BspSPI = {
    .Init = BspSPI0_Init,
    .TransByte = BspSPI0_TransByte,
    .ReceiveByte = BspSPI0_ReceiveByte,
    .TransMitByte = BspSPI0_TransMitByte,
    .TransMitBuff = BspSPI0_TransMitBuff,
};

static bool BspSPI_QuadMode_Init()
{
}

#pragma pack(1)
typedef struct
{
    GPIO_TypeDef *port_mosi;
    GPIO_TypeDef *port_miso;
    GPIO_TypeDef *port_clk;

    uint32_t pin_mosi;
    uint32_t pin_miso;
    uint32_t pin_clk;

    uint32_t pin_Alternate;
} BspSPI_PinConfig_TypeDef;

typedef struct
{
    BspSPI_PinConfig_TypeDef Pin;
    SPI_TypeDef *Instance;
    uint32_t Mode;
    uint32_t Direction;
    uint32_t DataSize;
    uint32_t CLKPolarity;
    uint32_t CLKPhase;
    uint32_t NSS;
    uint32_t BaudRatePrescaler;
    uint32_t FirstBit;
} BspSPI_NorModeConfig_TypeDef;

typedef struct
{
    SPI_TypeDef *Instance;
} BspSPI_QuadModeConfig_TypeDef;
#pragma pack()

static bool BspSPI_PinInit(BspSPI_PinConfig_TypeDef pin_cfg)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* mosi pin init */
    GPIO_InitStruct.Pin = pin_cfg.pin_mosi;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = pin_cfg.pin_Alternate;
    HAL_GPIO_Init(pin_cfg.port_mosi, &GPIO_InitStruct);

    /* miso pin init */
    GPIO_InitStruct.Pin = pin_cfg.pin_miso;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = pin_cfg.pin_Alternate;
    HAL_GPIO_Init(pin_cfg.port_miso, &GPIO_InitStruct);

    /* clk pin init */
    GPIO_InitStruct.Pin = pin_cfg.pin_clk;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = pin_cfg.pin_Alternate;
    HAL_GPIO_Init(pin_cfg.port_clk, &GPIO_InitStruct);
}

static bool BspSPI_NormalMode_Init(BspSPI_NorModeConfig_TypeDef spi_cfg)
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
    SPI_InitStructure.Init.Mode = spi_cfg.Mode;                           // SPI_MODE_MASTER;
    SPI_InitStructure.Init.Direction = spi_cfg.Direction;                 // SPI_DIRECTION_2LINES;
    SPI_InitStructure.Init.DataSize = spi_cfg.DataSize;                   // SPI_DATASIZE_8BIT;
    SPI_InitStructure.Init.CLKPolarity = spi_cfg.CLKPolarity;             // SPI_POLARITY_LOW;
    SPI_InitStructure.Init.CLKPhase = spi_cfg.CLKPhase;                   // SPI_PHASE_1EDGE;
    SPI_InitStructure.Init.NSS = spi_cfg.NSS;                             // SPI_NSS_SOFT;
    SPI_InitStructure.Init.BaudRatePrescaler = spi_cfg.BaudRatePrescaler; // SPI_BAUDRATEPRESCALER_2;
    SPI_InitStructure.Init.FirstBit = spi_cfg.FirstBit;                   // SPI_FIRSTBIT_MSB;
    SPI_InitStructure.Init.TIMode = SPI_TIMODE_DISABLE;
    SPI_InitStructure.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    SPI_InitStructure.Init.CRCPolynomial = 7;
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

    if (HAL_SPI_Init(SPI_InitStructure) == HAL_OK)
    {
        return false;
    }

    return true;
}

static bool BspSPI_DeInit(BspSPI_NorModeConfig_TypeDef spi_cfg)
{
}

static void BspSPI_TransByte(uint8_t tx)
{
}

static uint8_t BspSPI_ReceiveByte(void)
{
}

static uint8_t BspSPI_TransMitByte(uint8_t tx)
{
}

static uint16_t BspSPI_TransMitBuff(uint8_t *tx, uint8_t *rx, uint16_t size)
{
}
