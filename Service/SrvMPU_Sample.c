#include "SrvMPU_Sample.h"
#include "Bsp_SPI.h"
#include "Bsp_GPIO.h"
#include "Dev_MPU6000.h"

static SPI_HandleTypeDef MPU6000_SPI_Instance;
static BspGPIO_Obj_TypeDef MPU6000_CSPin_Cfg;
static BspSPI_NorModeConfig_TypeDef MPU6000_SPI_Cfg;

static SPI_HandleTypeDef ICM20602_SPI_Instance;
static BspSPI_NorModeConfig_TypeDef ICM20602_SPI_Cfg;
static BspGPIO_Obj_TypeDef ICM20602_CSPin_Cfg;
