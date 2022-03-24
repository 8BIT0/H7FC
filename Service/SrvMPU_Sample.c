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

static SPI_HandleTypeDef MPU6000_SPI_Instance;
static BspSPI_NorModeConfig_TypeDef MPU6000_SPI_Cfg;

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
static SPI_HandleTypeDef ICM20602_SPI_Instance;
static BspSPI_NorModeConfig_TypeDef ICM20602_SPI_Cfg;

/* internal function */

static bool SrvIMU_Init(void)
{
    BspGPIO.out_init(MPU6000_CSPin);
    BspGPIO.out_init(ICM20602_CSPin);
}
