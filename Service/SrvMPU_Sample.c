#include "SrvMPU_Sample.h"
#include "Bsp_SPI.h"
#include "Bsp_GPIO.h"
#include "Dev_MPU6000.h"
#include "IO_Definition.h"

/* internal variable */
/* MPU6000 Instance */
static BspGPIO_Obj_TypeDef MPU6000_CSPin = {
    .init_state = true,
    .pin = MPU6000_CS_PIN,
    .port = MPU6000_CS_PORT,
};

static BspGPIO_Obj_TypeDef MPU6000_INTPin = {
    .init_state = true,
    .pin = MPU6000_INT_PIN,
    .port = MPU6000_INT_PORT,
};

static SPI_HandleTypeDef MPU6000_Bus_Instance;
static BspSPI_PinConfig_TypeDef MPU6000_BusPin = {
    .pin_Alternate = GPIO_AF5_SPI1,

    .port_clk = MPU6000_CLK_PORT,
    .port_miso = MPU6000_MISO_PORT,
    .port_mosi = MPU6000_MOSI_PORT,

    .pin_clk = MPU6000_CLK_PIN,
    .pin_miso = MPU6000_MISO_PIN,
    .pin_mosi = MPU6000_MOSI_PIN,
};

static BspSPI_NorModeConfig_TypeDef MPU6000_BusCfg = {
    .Pin = MPU6000_BusPin,
    .Instance = MPU6000_SPI_BUS,
    .CLKPolarity = SPI_POLARITY_HIGH,
    .CLKPhase = SPI_PHASE_2EDGE,
    .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128,
);

/* ICM20602 Instance */
static BspGPIO_Obj_TypeDef ICM20602_CSPin = {
    .init_state = true,
    .pin = ICM20602_CS_PIN,
    .port = ICM20602_CS_PORT,
};

static BspGPIO_Obj_TypeDef ICM20602_INTPin = {
    .init_state = true,
    .pin = ICM20602_INT_PIN,
    .port = ICM20602_INT_PORT,
};

static SPI_HandleTypeDef ICM20602_Bus_Instance;
static BspSPI_PinConfig_TypeDef ICM20602_BusPin;
static BspSPI_NorModeConfig_TypeDef ICM20602_BusCfg = {
    .Pin = ICM20602_BusPin,
    .Instance = ICM20602_SPI_BUS,
    .CLKPolarity = SPI_POLARITY_HIGH,
    .CLKPhase = SPI_PHASE_2EDGE,
    .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128,
);

/* internal function */
static void SrvIMU_PriIMU_ExtiCallback(void);
static void SrvIMU_SecIMU_ExtiCallback(void);
static bool SrvIMU_Init(void);

static bool SrvIMU_Periph_Init(void)
{
    /* primary IMU Pin & Bus Init */
    BspGPIO.out_init(MPU6000_CSPin);
    BspGPIO.exti_init(MPU6000_INTPin, SrvIMU_PriIMU_ExtiCallback);
    BspSPI.init(MPU6000_BusCfg, &MPU6000_Bus_Instance);

    /* secondary IMU Pin & Bus Init */
    BspGPIO.out_init(ICM20602_CSPin);
    BspGPIO.exti_init(ICM20602_INTPin, SrvIMU_SecIMU_ExtiCallback);
    BspSPI.init(ICM20602_BusCfg, &ICM20602_Bus_Instance);
}

static void SrvIMU_PriIMU_ExtiCallback(void)
{
    /* MPU6000 Sample */
}

static void SrvIMU_SecIMU_ExtiCallback(void)
{
    /* ICM20602 Sample */
}
