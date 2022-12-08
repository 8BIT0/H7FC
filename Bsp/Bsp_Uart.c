/*
 * coder : 8_B!T0
 * bref : default using uart dma transfer
 */
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_uart.h"
#include "Bsp_Uart.h"

static bool BspUart_Init(BspUARTObj_TypeDef *obj)
{
    UART_HandleTypeDef uart_cfg;

    uart_cfg.Instance = obj->hdl.Instance;
    uart_cfg.Init.BaudRate = obj->baudrate;
    uart_cfg.Init.WordLength = UART_WORDLENGTH_8B;
    uart_cfg.Init.StopBits = UART_STOPBITS_1;
    uart_cfg.Init.Parity = UART_PARITY_NONE;
    uart_cfg.Init.Mode = UART_MODE_TX_RX;
    uart_cfg.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart_cfg.Init.OverSampling = UART_OVERSAMPLING_16;
    uart_cfg.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    uart_cfg.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    uart_cfg.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT | UART_ADVFEATURE_DMADISABLEONERROR_INIT;
    uart_cfg.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
    uart_cfg.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;

    if(obj->pin_swap)
    {
        /* swap tx rx */
        uart_cfg.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
    }

    if (HAL_UART_Init(&uart_cfg) != HAL_OK ||
        HAL_UARTEx_SetTxFifoThreshold(&uart_cfg, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK ||
        HAL_UARTEx_SetRxFifoThreshold(&uart_cfg, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK ||
        HAL_UARTEx_DisableFifoMode(&uart_cfg) != HAL_OK)
    {
        return false;
    }

    return true;
}
