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
#define PriIMU_SPI_BUS SPI1

#define PriIMU_CS_PORT GPIOC
#define PriIMU_CS_PIN GPIO_PIN_15

#define PriIMU_INT_PORT GPIOB
#define PriIMU_INT_PIN GPIO_PIN_2

#define PriIMU_CLK_PORT GPIOA
#define PriIMU_CLK_PIN GPIO_PIN_5

#define PriIMU_MISO_PORT GPIOA
#define PriIMU_MISO_PIN GPIO_PIN_6

#define PriIMU_MOSI_PORT GPIOD
#define PriIMU_MOSI_PIN GPIO_PIN_7

/* ICM20602 Pin */
#define SecIMU_SPI_BUS SPI4

#define SecIMU_CS_PORT GPIOC
#define SecIMU_CS_PIN GPIO_PIN_13

#define SecIMU_INT_PORT GPIOC
#define SecIMU_INT_PIN GPIO_PIN_14

#define SecIMU_CLK_PORT GPIOE
#define SecIMU_CLK_PIN GPIO_PIN_12

#define SecIMU_MISO_PORT GPIOE
#define SecIMU_MISO_PIN GPIO_PIN_13

#define SecIMU_MOSI_PORT GPIOE
#define SecIMU_MOSI_PIN GPIO_PIN_14

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

/* PWM IO */
#define PWM_SIG_1_PORT GPIOB
#define PWM_SIG_1_PIN GPIO_PIN_0

#define PWM_SIG_2_PORT GPIOB
#define PWM_SIG_2_PIN GPIO_PIN_1

#define PWM_SIG_3_PORT GPIOA
#define PWM_SIG_3_PIN GPIO_PIN_0

#define PWM_SIG_4_PORT GPIOA
#define PWM_SIG_4_PIN GPIO_PIN_1

#define PWM_SIG_5_PORT GPIOA
#define PWM_SIG_5_PIN GPIO_PIN_2

#define PWM_SIG_6_PORT GPIOA
#define PWM_SIG_6_PIN GPIO_PIN_3

#define PWM_SIG_7_PORT GPIOD
#define PWM_SIG_7_PIN GPIO_PIN_12

#define PWM_SIG_8_PORT GPIOD
#define PWM_SIG_8_PIN GPIO_PIN_13

#define PWM_SIG_9_PORT GPIOD
#define PWM_SIG_9_PIN GPIO_PIN_14

#define PWM_SIG_10_PORT GPIOD
#define PWM_SIG_10_PIN GPIO_PIN_15

#define PWM_SIG_11_PORT GPIOE
#define PWM_SIG_11_PIN GPIO_PIN_5

#define PWM_SIG_12_PORT GPIOE
#define PWM_SIG_12_PIN GPIO_PIN_6

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
extern BspGPIO_Obj_TypeDef PriIMU_CSPin;
extern BspGPIO_Obj_TypeDef SecIMU_CSPin;
extern BspGPIO_Obj_TypeDef PriIMU_INTPin;
extern BspGPIO_Obj_TypeDef SecIMU_INTPin;
extern BspGPIO_Obj_TypeDef Uart4_TxPin;
extern BspGPIO_Obj_TypeDef Uart4_RxPin;

extern BspSPI_PinConfig_TypeDef PriIMU_BusPin;
extern BspSPI_PinConfig_TypeDef SecIMU_BusPin;

#endif