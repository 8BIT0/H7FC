#include "SrvMPU_Sample.h"
#include "Bsp_SPI.h"
#include "Drv_GPIO.h"
#include "Dev_MPU6000.h"

#define CS_PIN_MODE GPIO_MODE_OUTPUT_PP
#define CS_PIN_SPEED GPIO_SPEED_FREQ_HIGH
#define CS_PIN_PULL GPIO_PULLUP
#define CS_DEFAULT_LEVEL GPIO_PIN_SET

/* MPU6000 Pin */
#define MPU6000_CS_PORT GPIOC
#define MPU6000_CS_PIN GPIO_PIN_15

#define GYRO_1_SPI_BUS SPI1
#define SPI1_SCK_PIN PA5
#define SPI1_MISO_PIN PA6
#define SPI1_MOSI_PIN PD7
#define GYRO_1_EXTI_PIN PB2

/* ICM20602 Pin */
#define ICM20602_CS_PORT GPIOE
#define ICM20602_CS_PIN GPIO_PIN_11

#define GYRO_2_SPI_BUS SPI4
#define SPI4_SCK_PIN PE12
#define SPI4_MISO_PIN PE13
#define SPI4_MOSI_PIN PE14
#define GYRO_2_EXTI_PIN PE15

/* internal variable */
static DrvGenGPIOObj_TypeDef MPU6000_CSPin_Cfg;
static SPI_HandleTypeDef MPU6000_SPI_Instance;
static BspSPI_NorModeConfig_TypeDef MPU6000_SPI_Cfg;

static DrvGenGPIOObj_TypeDef ICM20602_CSPin_Cfg;
static SPI_HandleTypeDef ICM20602_SPI_Instance;
static BspSPI_NorModeConfig_TypeDef ICM20602_SPI_Cfg;

/* internal function */
static void SrvIMU_CS_Config(DrvGenGPIOObj_TypeDef Obj);

static void SrvIMU_CS_Config(DrvGenGPIOObj_TypeDef Obj)
{
}

static bool SrvIMJU_PreInit(void)
{
}
