#ifndef __IO_DEFINITION_H
#define __IO_DEFINITION_H

#include "Bsp_GPIO.h"
#include "Bsp_SPI.h"
#include "Dev_Led.h"
#include "debug_util.h"

#define LED1_PIN GPIO_PIN_3
#define LED1_PORT GPIOE

#define LED2_PIN GPIO_PIN_4
#define LED2_PORT GPIOE

#define LED3_PIN GPIO_PIN_7
#define LED3_PORT GPIOH

/* SPI3 Reserve SPI */
#define RESERVE_SPI_CLK_PORT GPIOB
#define RESERVE_SPI_CLK_PIN GPIO_PIN_3

#define RESERVE_SPI_MISO_PORT GPIOB
#define RESERVE_SPI_MISO_PIN GPIO_PIN_4

#define RESERVE_SPI_MOSI_PORT GPIOB
#define RESERVE_SPI_MOSI_PIN GPIO_PIN_5

/* MPU6000 Pin */
#define MPU6000_SPI_BUS SPI1

#define MPU6000_CS_PORT GPIOC
#define MPU6000_CS_PIN GPIO_PIN_15

#define MPU6000_INT_PORT GPIOB
#define MPU6000_INT_PIN GPIO_PIN_2

#define MPU6000_CLK_PORT GPIOA
#define MPU6000_CLK_PIN GPIO_PIN_5

#define MPU6000_MISO_PORT GPIOA
#define MPU6000_MISO_PIN GPIO_PIN_6

#define MPU6000_MOSI_PORT GPIOD
#define MPU6000_MOSI_PIN GPIO_PIN_7

/* ICM20602 Pin */
#define ICM20602_SPI_BUS SPI4

#define ICM20602_CS_PORT GPIOE
#define ICM20602_CS_PIN GPIO_PIN_11

#define ICM20602_INT_PORT GPIOE
#define ICM20602_INT_PIN GPIO_PIN_15

#define ICM20602_CLK_PORT GPIOE
#define ICM20602_CLK_PIN GPIO_PIN_12

#define ICM20602_MISO_PORT GPIOE
#define ICM20602_MISO_PIN GPIO_PIN_13

#define ICM20602_MOSI_PORT GPIOE
#define ICM20602_MOSI_PIN GPIO_PIN_14

#define SDMMC_CLK_PORT GPIOC
#define SDMMC_CLK_PIN GPIO_PIN_12

#define SDMMC_CMD_PORT GPIOD
#define SDMMC_CMD_PIN GPIO_PIN_2

#define D0_PORT GPIOC
#define D0_PIN GPIO_PIN_8

#define D1_PORT GPIOC
#define D1_PIN GPIO_PIN_9

#define D2_PORT GPIOC
#define D2_PIN GPIO_PIN_10

#define D3_PORT GPIOC
#define D3_PIN GPIO_PIN_11

/* Serial Pin */
#define UART4_TX_PORT GPIOB
#define UART4_TX_PIN GPIO_PIN_9
#define UART4_RX_PORT GPIOB
#define UART4_RX_PIN GPIO_PIN_8

/* USB Detected Pin */
#define USB_DETECT_INT_PORT GPIOE
#define USB_DETECT_INT_PIN GPIO_PIN_2

extern DebugPinObj_TypeDef Debug_PC0;
extern DebugPinObj_TypeDef Debug_PC1;
extern DebugPinObj_TypeDef Debug_PC2;
extern DebugPinObj_TypeDef Debug_PC3;
extern DebugPinObj_TypeDef Debug_PB3;
extern DebugPinObj_TypeDef Debug_PB4;
extern DebugPinObj_TypeDef Debug_PB5;
extern DebugPinObj_TypeDef Debug_PB6;
extern DebugPinObj_TypeDef Debug_PB10;

extern DevLedObj_TypeDef Led1;
extern DevLedObj_TypeDef Led2;
extern DevLedObj_TypeDef Led3;

extern BspGPIO_Obj_TypeDef USB_DctPin;
extern BspGPIO_Obj_TypeDef MPU6000_CSPin;
extern BspGPIO_Obj_TypeDef ICM20602_CSPin;
extern BspGPIO_Obj_TypeDef MPU6000_INTPin;
extern BspGPIO_Obj_TypeDef ICM20602_INTPin;
extern BspGPIO_Obj_TypeDef Uart4_TxPin;
extern BspGPIO_Obj_TypeDef Uart4_RxPin;

extern BspSPI_PinConfig_TypeDef MPU6000_BusPin;
extern BspSPI_PinConfig_TypeDef ICM20602_BusPin;

#endif
