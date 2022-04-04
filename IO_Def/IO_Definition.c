#include "IO_Definition.h"
#include "Dev_Led.h"

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
