#include "HW_Def.h"

// MOTOR 1 B06
// MOTOR 2 B07
// MOTOR 3 A03
// MOTOR 4 B01

// SERIAL_TX 1 A09
// SERIAL_TX 2 A02
// SERIAL_TX 3 B11
// SERIAL_TX 5 B09
// SERIAL_TX 7 B04

// SERIAL_RX 1 A10
// SERIAL_RX 2 B00
// SERIAL_RX 3 B10
// SERIAL_RX 5 B08
// SERIAL_RX 7 B03

// I2C_SCL 2 H02
// I2C_SDA 2 H03

// LED 1 C13
// LED 2 C14

// SPI_SCK 1 A05
// SPI_MISO 1 A06
// SPI_MOSI 1 A07
// SPI_SCK 2 B13
// SPI_MISO 2 B14
// SPI_MOSI 2 B15

// FLASH_CS 1 B05
// OSD_CS 1 B12
// GYRO_EXTI 1 A15
// GYRO_CS 1 A04

// ADC_BATT 1 A00
// ADC_CURR 1 A01

// timer A08 AF1
// # pin A08: TIM1 CH1 (AF1)
// timer B06 AF2
// # pin B06: TIM4 CH1 (AF2)
// timer B07 AF2
// # pin B07: TIM4 CH2 (AF2)
// timer A03 AF1
// # pin A03: TIM2 CH4 (AF1)
// timer B01 AF2
// # pin B01: TIM3 CH4 (AF2)

// dma ADC 1 11
// # ADC 1: DMA2 Channel 0 Channel 5

// dma pin A08 7
// # pin A08: DMA2 Channel 0 Channel 42
// dma pin B06 0
// # pin B06: DMA1 Channel 0 Channel 67
// dma pin B07 1
// # pin B07: DMA1 Channel 0 Channel 68
// dma pin A03 2
// # pin A03: DMA1 Channel 0 Channel 59
// dma pin B01 3
// # pin B01: DMA1 Channel 0 Channel 64

/* Led 1 */
BspGPIO_Port_TypeDef Led1_Port = {
    .port = GPIOC,
};

BspGPIO_Obj_TypeDef Led1 = {
    .port = (void *)&Led1_Port,
    .pin = GPIO_PINS_13,
    .init_state = GPIO_PULL_NONE,
};

/* Led 2 */
BspGPIO_Port_TypeDef Led2_Port = {
    .port = GPIOC,
};

BspGPIO_Obj_TypeDef Led1 = {
    .port = (void *)&Led1_Port,
    .pin = GPIO_PINS_14,
    .init_state = GPIO_PULL_NONE,
};

/* on board receiver uart tx port */
BspGPIO_Port_TypeDef Uart3_Tx_Port = {
    .port = GPIOB,
};

BspGPIO_Obj_TypeDef Uart3_TxPin = {
    .port = (void *)&Uart3_Tx_Port,
    .pin = GPIO_PINS_10,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_7,
};

/* on board receiver uart rx port */
BspGPIO_Port_TypeDef Uart3_Rx_Port = {
    .port = GPIOB,
};

BspGPIO_Obj_TypeDef Uart3_RxPin = {
    .port = (void *)&Uart3_Rx_Port,
    .pin = GPIO_PINS_11,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_7,
};

/* uart 1 tx pin */
BspGPIO_Port_TypeDef Uart1_Tx_Port = {
    .port = GPIOA,
};

BspGPIO_Obj_TypeDef Uart1_TxPin = {
    .port = (void *)&Uart1_Tx_Port,
    .pin = GPIO_PINS_9,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_7,
};

/* uart 1 rx pin */
BspGPIO_Port_TypeDef Uart1_Rx_Port = {
    .port = GPIOA,
};

BspGPIO_Obj_TypeDef Uart1_RxPin = {
    .port = (void *)&Uart1_Rx_Port,
    .pin = GPIO_PINS_10,
    .init_state = GPIO_PULL_NONE,
    .alternate = GPIO_MUX_7,
};

