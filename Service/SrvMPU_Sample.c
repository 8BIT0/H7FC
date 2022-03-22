#include "SrvMPU_Sample.h"
#include "Bsp_SPI.h"
#include "Dev_MPU6000.h"

static SPI_HandleTypeDef MPU6000_SPI;
static SPI_HandleTypeDef ICM20602_SPI;

static bool SrvIMU_Init(void)
{
}
