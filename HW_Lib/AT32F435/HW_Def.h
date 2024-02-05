#ifndef __HW_DEF_H
#define __HW_DEF_H

#include "Bsp_GPIO.h"
#include "Bsp_DMA.h"
#include "Bsp_Uart.h"
#include "Bsp_IIC.h"
#include "Bsp_SPI.h"
#include "Bsp_Flash.h"
#include "Dev_Led.h"
#include "debug_util.h"
#include "util.h"

/* device support */
#include "Dev_W25Qxx.h"

#define RECEIVER_PORT BspUART_Port_4
#define RECEIVER_CRSF_RX_DMA Bsp_DMA_None               // Bsp_DMA_1
#define RECEIVER_CRSF_RX_DMA_STREAM Bsp_DMA_Stream_None // Bsp_DMA_Stream_4
#define RECEIVER_CRSF_TX_DMA Bsp_DMA_None               // Bsp_DMA_1
#define RECEIVER_CRSF_TX_DMA_STREAM Bsp_DMA_Stream_None // Bsp_DMA_Stream_5

#define RECEIVER_SBUS_RX_DMA Bsp_DMA_1
#define RECEIVER_SBUS_RX_DMA_STREAM Bsp_DMA_Stream_5
#define RECEIVER_SBUS_TX_DMA Bsp_DMA_1
#define RECEIVER_SBUS_TX_DMA_STREAM Bsp_DMA_Stream_6

#define CRSF_TX_PIN Uart3_TxPin
#define CRSF_RX_PIN Uart3_RxPin

#define SBUS_TX_PIN Uart4_TxPin
#define SBUS_RX_PIN Uart4_RxPin

/* radio port */
#define RADIO_PORT USART1
#define RADIO_TX_PIN_INIT_STATE GPIO_PULL_NONE
#define RADIO_RX_PIN_INIT_STATE GPIO_PULL_NONE
#define RADIO_TX_PIN_ALT GPIO_MUX_7
#define RADIO_RX_PIN_ALT GPIO_MUX_7
#define RADIO_TX_DMA Bsp_DMA_2
#define RADIO_TX_DMA_STREAM Bsp_DMA_Stream_1
#define RADIO_RX_DMA Bsp_DMA_2
#define RADIO_RX_DMA_STREAM Bsp_DMA_Stream_2
#define RADIO_TX_PIN GPIO_PINS_9
#define RADIO_RX_PIN GPIO_PINS_10
#define RADIO_TX_PORT &Uart1_Tx_Port
#define RADIO_RX_PORT &Uart1_Rx_Port

/* IMU SPI */
#define PriIMU_SPI_BUS SPI1

/* PWM IO */
#define PWM_SIG_1_TIM TMR3
#define PWM_SIG_1_TIM_CHANNEL TMR_SELECT_CHANNEL_3
#define PWM_SIG_1_PORT GPIOB
#define PWM_SIG_1_PIN GPIO_PINS_0
#define PWM_SIG_1_DMA Bsp_DMA_1
#define PWM_SIG_1_DMA_CHANNEL Bsp_DMA_Stream_1
#define PWM_SIG_1_PIN_AF GPIO_MUX_2

#define PWM_SIG_2_TIM TMR3
#define PWM_SIG_2_TIM_CHANNEL TMR_SELECT_CHANNEL_4
#define PWM_SIG_2_PORT GPIOB
#define PWM_SIG_2_PIN GPIO_PINS_1
#define PWM_SIG_2_DMA Bsp_DMA_1
#define PWM_SIG_2_DMA_CHANNEL Bsp_DMA_Stream_2
#define PWM_SIG_2_PIN_AF GPIO_MUX_2

#define PWM_SIG_3_TIM TMR2
#define PWM_SIG_3_TIM_CHANNEL TMR_SELECT_CHANNEL_4
#define PWM_SIG_3_PORT GPIOA
#define PWM_SIG_3_PIN GPIO_PINS_3
#define PWM_SIG_3_DMA Bsp_DMA_1
#define PWM_SIG_3_DMA_CHANNEL Bsp_DMA_Stream_3
#define PWM_SIG_3_PIN_AF GPIO_MUX_1

#define PWM_SIG_4_TIM TMR2
#define PWM_SIG_4_TIM_CHANNEL TMR_SELECT_CHANNEL_3
#define PWM_SIG_4_PORT GPIOA
#define PWM_SIG_4_PIN GPIO_PINS_2
#define PWM_SIG_4_DMA Bsp_DMA_1
#define PWM_SIG_4_DMA_CHANNEL Bsp_DMA_Stream_4
#define PWM_SIG_4_PIN_AF GPIO_MUX_1

#define PWM_SIG_5_TIM TMR8
#define PWM_SIG_5_TIM_CHANNEL TMR_SELECT_CHANNEL_3
#define PWM_SIG_5_PORT GPIOC
#define PWM_SIG_5_PIN GPIO_PINS_8
#define PWM_SIG_5_DMA Bsp_DMA_None
#define PWM_SIG_5_DMA_CHANNEL Bsp_DMA_Stream_None
#define PWM_SIG_5_PIN_AF GPIO_MUX_3

#define PWM_SIG_6_TIM TMR1
#define PWM_SIG_6_TIM_CHANNEL TMR_SELECT_CHANNEL_1
#define PWM_SIG_6_PORT GPIOA
#define PWM_SIG_6_PIN GPIO_PINS_8
#define PWM_SIG_6_DMA Bsp_DMA_None
#define PWM_SIG_6_DMA_CHANNEL Bsp_DMA_Stream_None 
#define PWM_SIG_6_PIN_AF GPIO_MUX_1

#define PWM_SIG_7_TIM TMR4
#define PWM_SIG_7_TIM_CHANNEL TMR_SELECT_CHANNEL_1
#define PWM_SIG_7_PORT GPIOB
#define PWM_SIG_7_PIN GPIO_PINS_6
#define PWM_SIG_7_DMA Bsp_DMA_None
#define PWM_SIG_7_DMA_CHANNEL Bsp_DMA_Stream_None
#define PWM_SIG_7_PIN_AF GPIO_MUX_2

/* internal flash storage */
#define OnChipFlash_Storage_StartAddress FLASH_BLOCK_7_START_ADDR
#define OnChipFlash_Storage_TotalSize FLASH_BLOCK_7_SIZE
#define OnChipFlash_Storage_DefaultData FLASH_DEFAULT_DATA

#define OnChipFlash_MaxRWSize (2 Kb)
#define OnChipFlash_Storage_TabSize (4 Kb)
#define OnChipFlash_Storage_InfoPageSize (1 Kb)

/* external flash storage */
#if (FLASH_CHIP_STATE == ON)
#define ExtFlash_Bus_Type Storage_ChipBus_Spi
#define ExtFlash_Bus_Clock_Div SPI_MCLK_DIV_32
#define ExtFlash_Chip_Type Storage_ChipType_W25Qxx
#define ExtFlash_Bus_Api BspSPI
#define ExtFLash_Bus_Instance (void *)SPI2
#define ExtFlash_Bus_CLKPhase SPI_CLOCK_PHASE_2EDGE
#define ExtFlash_Bus_CLKPolarity SPI_CLOCK_POLARITY_HIGH
#define ExtFlash_CS_Pin ExtFlash_CSPin
#define ExtFlash_Bus_Pin ExtFlash_SPIPin

#define ExtFlash_Dev_Api (void *)(&DevW25Qxx)
#define ExtFlash_Start_Addr W25QXX_BASE_ADDRESS

#define ExtFlash_Storage_DefaultData FLASH_DEFAULT_DATA
#define ExtFlash_Storage_TotalSize (2 Mb)
#define ExtFlash_Storage_TabSize (4 Kb)
#define ExtFlash_Storage_InfoPageSize (1 Kb)

extern BspGPIO_Obj_TypeDef ExtFlash_CSPin;
extern BspSPI_PinConfig_TypeDef ExtFlash_SPIPin;
#endif

extern BspGPIO_Port_TypeDef Uart3_Tx_Port;
extern BspGPIO_Port_TypeDef Uart3_Rx_Port;
extern BspGPIO_Port_TypeDef Uart1_Tx_Port;
extern BspGPIO_Port_TypeDef Uart1_Rx_Port;
extern BspGPIO_Port_TypeDef Uart4_Tx_Port;
extern BspGPIO_Port_TypeDef Uart5_Rx_Port;

extern BspGPIO_Obj_TypeDef Uart3_TxPin;
extern BspGPIO_Obj_TypeDef Uart3_RxPin;
extern BspGPIO_Obj_TypeDef Uart1_TxPin;
extern BspGPIO_Obj_TypeDef Uart1_RxPin;
extern BspGPIO_Obj_TypeDef Uart4_TxPin;
extern BspGPIO_Obj_TypeDef Uart4_RxPin;

extern BspGPIO_Obj_TypeDef PriIMU_CSPin;
extern BspGPIO_Obj_TypeDef PriIMU_INTPin;
extern DevLedObj_TypeDef Led1;
extern BspSPI_PinConfig_TypeDef PriIMU_BusPin;
extern BspSPI_NorModeConfig_TypeDef PriIMU_BusCfg;

#endif
