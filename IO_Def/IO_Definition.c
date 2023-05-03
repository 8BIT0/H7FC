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

BspGPIO_Obj_TypeDef PriIMU_CSPin = {
    .init_state = true,
    .pin = PriIMU_CS_PIN,
    .port = PriIMU_CS_PORT,
};

BspGPIO_Obj_TypeDef PriIMU_INTPin = {
    .init_state = true,
    .pin = PriIMU_INT_PIN,
    .port = PriIMU_INT_PORT,
};

BspGPIO_Obj_TypeDef SecIMU_CSPin = {
    .init_state = true,
    .pin = SecIMU_CS_PIN,
    .port = SecIMU_CS_PORT,
};

BspGPIO_Obj_TypeDef SecIMU_INTPin = {
    .init_state = true,
    .pin = SecIMU_INT_PIN,
    .port = SecIMU_INT_PORT,
};

BspGPIO_Obj_TypeDef Uart4_TxPin = {
    .pin = UART4_TX_PIN,
    .port = UART4_TX_PORT,
    .alternate = GPIO_AF8_UART4,
};

BspGPIO_Obj_TypeDef Uart4_RxPin = {
    .pin = UART4_RX_PIN,
    .port = UART4_RX_PORT,
    .alternate = GPIO_AF8_UART4,
};

BspSPI_PinConfig_TypeDef PriIMU_BusPin = {
    .pin_Alternate = GPIO_AF5_SPI1,

    .port_clk = PriIMU_CLK_PORT,
    .port_miso = PriIMU_MISO_PORT,
    .port_mosi = PriIMU_MOSI_PORT,

    .pin_clk = PriIMU_CLK_PIN,
    .pin_miso = PriIMU_MISO_PIN,
    .pin_mosi = PriIMU_MOSI_PIN,
};

BspSPI_PinConfig_TypeDef SecIMU_BusPin = {
    .pin_Alternate = GPIO_AF5_SPI4,

    .port_clk = SecIMU_CLK_PORT,
    .port_miso = SecIMU_MISO_PORT,
    .port_mosi = SecIMU_MOSI_PORT,

    .pin_clk = SecIMU_CLK_PIN,
    .pin_miso = SecIMU_MISO_PIN,
    .pin_mosi = SecIMU_MOSI_PIN,
};

BspGPIO_Obj_TypeDef USB_DctPin = {
    .init_state = false,
    .pin = USB_DETECT_INT_PIN,
    .port = USB_DETECT_INT_PORT,
};
