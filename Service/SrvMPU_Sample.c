#include "SrvMPU_Sample.h"
#include "Dev_MPU6000.h"
#include "IO_Definition.h"
#include "runtime.h"
#include "debug_util.h"
#include "Bsp_SPI.h"
#include "error_log.h"

/*
 *   PriIMU -> MPU6000
 *   SecIMU -> ICM20602
 */

#define IMU_Commu_TimeOut 1000

/* internal variable */
/* MPU6000 Instance */
static SPI_HandleTypeDef MPU6000_Bus_Instance;
static BspSPI_NorModeConfig_TypeDef MPU6000_BusCfg = {
    .Instance = MPU6000_SPI_BUS,
    .CLKPolarity = SPI_POLARITY_HIGH,
    .CLKPhase = SPI_PHASE_2EDGE,
    .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128,
};

/* ICM20602 Instance */
static SPI_HandleTypeDef ICM20602_Bus_Instance;
static BspSPI_NorModeConfig_TypeDef ICM20602_BusCfg = {
    .Instance = ICM20602_SPI_BUS,
    .CLKPolarity = SPI_POLARITY_HIGH,
    .CLKPhase = SPI_PHASE_2EDGE,
    .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128,
};

static DevMPU6000Obj_TypeDef MPU6000Obj;

/* internal function */
static int8_t SrvIMU_PriIMU_Init(void);
static int8_t SrvIMU_SecIMU_Init(void);
static void SrvIMU_PriIMU_ExtiCallback(void);
static void SrvIMU_SecIMU_ExtiCallback(void);
static void SrvIMU_PriIMU_CS_Ctl(bool state);
static void SrvIMU_SecIMU_CS_Ctl(bool state);
static bool SrvIMU_PriIMU_BusTrans_Rec(uint8_t *Tx, uint8_t *Rx, uint16_t size);
static bool SrvIMU_SecIMU_BusTrans_Rec(uint8_t *Tx, uint8_t *Rx, uint16_t size);

SrvIMU_ErrorCode_List SrvIMU_Init(void)
{
    SrvIMU_ErrorCode_List PriIMU_Init_State = SrvIMU_PriIMU_Init();
    SrvIMU_ErrorCode_List SecIMU_Init_State = SrvIMU_SecIMU_Init();

    // error log

    if ((PriIMU_Init_State != SrvIMU_No_Error) && (SecIMU_Init_State != SrvIMU_No_Error))
    {
        return SrvIMU_Sample_Init_Error;
    }

    return SrvIMU_No_Error;
}

/* init primary IMU Device */
static SrvIMU_ErrorCode_List SrvIMU_PriIMU_Init(void)
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
                        Runtime_DelayMs,
                        Get_CurrentRunningUs);

    DevMPU6000.config(&MPU6000Obj,
                      DevMPU6000_SampleRate_4K,
                      MPU6000_Acc_16G,
                      MPU6000_Gyr_2000DPS);

    if (!BspSPI.init(MPU6000_BusCfg, &MPU6000_Bus_Instance))
        return SrvIMU_PriBus_Init_Error;

    if (!DevMPU6000.init(&MPU6000Obj))
        return SrvIMU_PriDev_Init_Error;

    /* Set SPI Speed 20M */
    MPU6000_BusCfg.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;

    return SrvIMU_No_Error;
}

/* init primary IMU Device */
static SrvIMU_ErrorCode_List SrvIMU_SecIMU_Init(void)
{
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
    DevMPU6000.set_drdy(&MPU6000Obj);
}

static void SrvIMU_SecIMU_ExtiCallback(void)
{
    /* ICM20602 Sample */
}

int8_t SrvIMU_GetPri_InitError(void)
{
    return DevMPU6000.get_error(&MPU6000Obj);
}
