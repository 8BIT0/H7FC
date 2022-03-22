#include "SrvMPU_Sample.h"
#include "Bsp_SPI.h"
#include "Bsp_GPIO.h"
#include "Dev_MPU6000.h"

/* MPU6000 Pin */
#define GYRO_1_SPI_BUS SPI1
#define SPI1_SCK_PIN PA5
#define SPI1_MISO_PIN PA6
#define SPI1_MOSI_PIN PD7
#define GYRO_1_CS_PIN PC15
#define GYRO_1_EXTI_PIN PB2

/* ICM20602 Pin */
#define GYRO_2_SPI_BUS SPI4
#define SPI4_SCK_PIN PE12
#define SPI4_MISO_PIN PE13
#define SPI4_MOSI_PIN PE14
#define GYRO_2_CS_PIN PE11
#define GYRO_2_EXTI_PIN PE15

static BspGPIO_Obj_TypeDef MPU6000_CSPin_Cfg;
static SPI_HandleTypeDef MPU6000_SPI_Instance;
static BspSPI_NorModeConfig_TypeDef MPU6000_SPI_Cfg;

static BspGPIO_Obj_TypeDef ICM20602_CSPin_Cfg;
static SPI_HandleTypeDef ICM20602_SPI_Instance;
static BspSPI_NorModeConfig_TypeDef ICM20602_SPI_Cfg;
