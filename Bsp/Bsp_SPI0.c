#include "Bsp_SPI0.h"
#include "stm32h7xx.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal_spi.h"

/* internal function */
static bool BspSPI0_Init();
static void BspSPI0_TransByte(uint8_t tx);
static uint8_t BspSPI0_ReceiveByte(void);
static uint8_t BspSPI0_TransMitByte(uint8_t tx);
static uint16_t BspSPI0_TransMitBuff(uint8_t *tx, uint8_t *rx, uint16_t size);

/* SPI0 Object */
BspSpi_TypeDef BspSPI0 = {
    .Init = BspSPI0_Init,
    .TransByte = BspSPI0_TransByte,
    .ReceiveByte = BspSPI0_ReceiveByte,
    .TransMitByte = BspSPI0_TransMitByte,
    .TransMitBuff = BspSPI0_TransMitBuff,
};

static bool BspSPI0_Init()
{
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 0x0;
    hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;

    HAL_SPI_Init(hspi);

    return true;
}

static void BspSPI0_TransByte(uint8_t tx)
{
}

static uint8_t BspSPI0_ReceiveByte(void)
{
}

static uint8_t BspSPI0_TransMitByte(uint8_t tx)
{
}

static uint16_t BspSPI0_TransMitBuff(uint8_t *tx, uint8_t *rx, uint16_t size)
{
}
