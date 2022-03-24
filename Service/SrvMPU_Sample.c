#include "SrvMPU_Sample.h"
#include "Bsp_SPI.h"
#include "Bsp_GPIO.h"
#include "Dev_MPU6000.h"

#define CS_PIN_MODE GPIO_MODE_OUTPUT_PP
#define CS_PIN_SPEED GPIO_SPEED_FREQ_HIGH
#define CS_PIN_PULL GPIO_PULLUP
#define CS_DEFAULT_LEVEL GPIO_PIN_SET

/* MPU6000 Pin */
#define GYRO_1_SPI_BUS SPI1
#define SPI1_SCK_PIN PA5
#define SPI1_MISO_PIN PA6
#define SPI1_MOSI_PIN PD7

/* ICM20602 Pin */
#define GYRO_2_SPI_BUS SPI4
#define SPI4_SCK_PIN PE12
#define SPI4_MISO_PIN PE13
#define SPI4_MOSI_PIN PE14

/* internal variable */
/* MPU6000 Instance */
static BspGPIO_Obj_TypeDef MPU6000_CSPin = {
    .init_state = true,
    .pin = GPIO_PIN_15,
    .port = GPIOC,
};

static BspGPIO_Obj_TypeDef MPU6000_INTPin = {
    .init_state = true,
    .pin = GPIO_PIN_2,
    .port = GPIOB,
};

static SPI_HandleTypeDef MPU6000_SPI_Instance;
static BspSPI_NorModeConfig_TypeDef MPU6000_SPI_Cfg;

/* ICM20602 Instance */
static BspGPIO_Obj_TypeDef ICM20602_CSPin = {
    .init_state = true,
    .pin = GPIO_PIN_11,
    .port = GPIOE,
};

static BspGPIO_Obj_TypeDef ICM20602_INTPin = {
    .init_state = true,
    .pin = GPIO_PIN_15,
    .port = GPIOE,
};
static SPI_HandleTypeDef ICM20602_SPI_Instance;
static BspSPI_NorModeConfig_TypeDef ICM20602_SPI_Cfg;

/* internal function */

static bool SrvIMU_Init(void)
{
}
