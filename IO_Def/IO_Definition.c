#include "IO_Definition.h"
#include "Dev_Led.h"

DebugPinObj_TypeDef Debug_PC0 = {
    .port = GPIOC,
    .pin = GPIO_PIN_0,
    .init_state = false,
};

DebugPinObj_TypeDef Debug_PC1 = {
    .port = GPIOC,
    .pin = GPIO_PIN_1,
    .init_state = false,
};

DebugPinObj_TypeDef Debug_PC2 = {
    .port = GPIOC,
    .pin = GPIO_PIN_2,
    .init_state = false,
};

DebugPinObj_TypeDef Debug_PC3 = {
    .port = GPIOC,
    .pin = GPIO_PIN_3,
    .init_state = false,
};

DebugPinObj_TypeDef Debug_PB3 = {
    .port = GPIOB,
    .pin = GPIO_PIN_3,
    .init_state = false,
};

DebugPinObj_TypeDef Debug_PB4 = {
    .port = GPIOB,
    .pin = GPIO_PIN_4,
    .init_state = false,
};

DebugPinObj_TypeDef Debug_PB5 = {
    .port = GPIOB,
    .pin = GPIO_PIN_5,
    .init_state = false,
};

DebugPinObj_TypeDef Debug_PB6 = {
    .port = GPIOB,
    .pin = GPIO_PIN_6,
    .init_state = false,
};

DebugPinObj_TypeDef Debug_PB10 = {
    .port = GPIOB,
    .pin = GPIO_PIN_10,
    .init_state = false,
};

DevLedObj_TypeDef Led1 = {
    .port = LED1_PORT,
    .pin = LED1_PIN,
    .init_state = true,
};

DevLedObj_TypeDef Led2 = {
    .port = LED2_PORT,
    .pin = LED2_PIN,
    .init_state = true,
};

DevLedObj_TypeDef Led3 = {
    .port = LED3_PORT,
    .pin = LED3_PIN,
    .init_state = true,
};

BspGPIO_Obj_TypeDef MPU6000_CSPin = {
    .init_state = true,
    .pin = MPU6000_CS_PIN,
    .port = MPU6000_CS_PORT,
};

BspGPIO_Obj_TypeDef MPU6000_INTPin = {
    .init_state = true,
    .pin = MPU6000_INT_PIN,
    .port = MPU6000_INT_PORT,
};

BspGPIO_Obj_TypeDef ICM20602_CSPin = {
    .init_state = true,
    .pin = ICM20602_CS_PIN,
    .port = ICM20602_CS_PORT,
};

BspGPIO_Obj_TypeDef ICM20602_INTPin = {
    .init_state = true,
    .pin = ICM20602_INT_PIN,
    .port = ICM20602_INT_PORT,
};

BspGPIO_Obj_TypeDef Uart4_TxPin = {
    .init_state = true,
    .pin = ,
    .port = ,
    .alternate = GPIO_AF8_UART4,
};

BspGPIO_Obj_TypeDef Uart4_RxPin = {
    .init_state = true,
    .pin = ,
    .port = ,
    .alternate = GPIO_AF8_UART4;
};

BspSPI_PinConfig_TypeDef MPU6000_BusPin = {
    .pin_Alternate = GPIO_AF5_SPI1,

    .port_clk = MPU6000_CLK_PORT,
    .port_miso = MPU6000_MISO_PORT,
    .port_mosi = MPU6000_MOSI_PORT,

    .pin_clk = MPU6000_CLK_PIN,
    .pin_miso = MPU6000_MISO_PIN,
    .pin_mosi = MPU6000_MOSI_PIN,
};

BspSPI_PinConfig_TypeDef ICM20602_BusPin = {
    .pin_Alternate = GPIO_AF5_SPI4,

    .port_clk = ICM20602_CLK_PORT,
    .port_miso = ICM20602_MISO_PORT,
    .port_mosi = ICM20602_MOSI_PORT,

    .pin_clk = ICM20602_CLK_PIN,
    .pin_miso = ICM20602_MISO_PIN,
    .pin_mosi = ICM20602_MOSI_PIN,
};

BspGPIO_Obj_TypeDef USB_DctPin = {
    .init_state = false,
    .pin = USB_DETECT_INT_PIN,
    .port = USB_DETECT_INT_PORT,
};
