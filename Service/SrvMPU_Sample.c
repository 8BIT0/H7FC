#include "SrvMPU_Sample.h"
#include "Bsp_SPI.h"
#include "Bsp_GPIO.h"
#include "Dev_MPU6000.h"
#include "IO_Definition.h"
#include "runtime.h"

/*
 *   PriIMU -> MPU6000
 *   SecIMU -> ICM20602
 */

#define IMU_Commu_TimeOut 1000

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
    .Instance = MPU6000_SPI_BUS,
    .CLKPolarity = SPI_POLARITY_HIGH,
    .CLKPhase = SPI_PHASE_2EDGE,
    .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128,
};

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
static BspSPI_PinConfig_TypeDef ICM20602_BusPin = {
    .pin_Alternate = GPIO_AF5_SPI4,

    .port_clk = ICM20602_CLK_PORT,
    .port_miso = ICM20602_MISO_PORT,
    .port_mosi = ICM20602_MOSI_PORT,

    .pin_clk = ICM20602_CLK_PIN,
    .pin_miso = ICM20602_MISO_PIN,
    .pin_mosi = ICM20602_MOSI_PIN,
};

static BspSPI_NorModeConfig_TypeDef ICM20602_BusCfg = {
    .Instance = ICM20602_SPI_BUS,
    .CLKPolarity = SPI_POLARITY_HIGH,
    .CLKPhase = SPI_PHASE_2EDGE,
    .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128,
};

static DevMPU6000Obj_TypeDef MPU6000Obj;

/* internal function */
static void SrvIMU_PriIMU_ExtiCallback(void);
static void SrvIMU_SecIMU_ExtiCallback(void);
static void SrvIMU_PriIMU_CS_Ctl(bool state);
static void SrvIMU_SecIMU_CS_Ctl(bool state);
static bool SrvIMU_PriIMU_BusTrans_Rec(uint8_t *Tx, uint8_t *Rx, uint16_t size);
static bool SrvIMU_SecIMU_BusTrans_Rec(uint8_t *Tx, uint8_t *Rx, uint16_t size);

int8_t SrvIMU_Init(void)
{
    /* primary IMU Pin & Bus Init */
    if (!BspGPIO.out_init(MPU6000_CSPin))
        return SrvIMU_PriCSPin_Init_Error;

    if (!BspGPIO.exti_init(MPU6000_INTPin, SrvIMU_PriIMU_ExtiCallback))
        return SrvIMU_PriExtiPin_Init_Error;

    MPU6000_BusCfg.Pin = MPU6000_BusPin;

    DevMPU6000.pre_init(&MPU6000Obj,
                        SrvIMU_PriIMU_CS_Ctl,
                        SrvIMU_PriIMU_BusTrans_Rec,
                        Runtime_DelayMs);

    if (!BspSPI.init(MPU6000_BusCfg, &MPU6000_Bus_Instance))
        return SrvIMU_PriBus_Init_Error;

    if (!DevMPU6000.init(&MPU6000Obj))
        return SrvIMU_PriDev_Init_Error;

    /* Set SPI Speed 20M */
    MPU6000_BusCfg.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;

    /* secondary IMU Pin & Bus Init */
    // if (!BspGPIO.out_init(ICM20602_CSPin))
    //     return false;

    // if (!BspGPIO.exti_init(ICM20602_INTPin, SrvIMU_SecIMU_ExtiCallback))
    //     return false;

    // if (!BspSPI.init(ICM20602_BusCfg, &ICM20602_Bus_Instance))
    //     return false;

    // MPU6000_BusCfg.Pin = ICM20602_BusPin;

    return SrvIMU_No_Error;
}

/* input true selected / false deselected */
static void SrvIMU_PriIMU_CS_Ctl(bool state)
{
    BspGPIO.write(MPU6000_CSPin.port, MPU6000_CSPin.pin, state);
}

static void SrvIMU_SecIMU_CS_Ctl(bool state)
{
    BspGPIO.write(ICM20602_CSPin.port, ICM20602_CSPin.pin, state);
}

static bool SrvIMU_PriIMU_BusTrans_Rec(uint8_t *Tx, uint8_t *Rx, uint16_t size)
{
    return BspSPI.trans_receive(&MPU6000_Bus_Instance, Tx, Rx, size, IMU_Commu_TimeOut);
}

static bool SrvIMU_SecIMU_BusTrans_Rec(uint8_t *Tx, uint8_t *Rx, uint16_t size)
{
    return BspSPI.trans_receive(&ICM20602_Bus_Instance, Tx, Rx, size, IMU_Commu_TimeOut);
}

static void SrvIMU_PriIMU_ExtiCallback(void)
{
    /* MPU6000 Sample */
}

static void SrvIMU_SecIMU_ExtiCallback(void)
{
    /* ICM20602 Sample */
}

int8_t SrvIMU_GetPri_InitError(void)
{
    return DevMPU6000.get_error(&MPU6000Obj);
}
