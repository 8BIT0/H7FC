#include "SrvMPU_Sample.h"
#include "Dev_MPU6000.h"
#include "Dev_ICM20602.h"
#include "IO_Definition.h"
#include "runtime.h"
#include "debug_util.h"
#include "Bsp_SPI.h"
#include "error_log.h"
#include "Dev_Led.h"

#define IMU_Commu_TimeOut 1000
#define MPU_MODULE_INIT_RETRY 10 // init retry count 10

/*
 *   PriIMU -> MPU6000
 *   SecIMU -> ICM20602
 */
static SrvMpu_InitReg_TypeDef SrvMpu_Reg;
static Error_Handler SrvMPU_Error_Handle = NULL;

/* internal variable */
/* MPU6000 Instance */
static SPI_HandleTypeDef MPU6000_Bus_Instance;
static BspSPI_NorModeConfig_TypeDef MPU6000_BusCfg = {
    .Instance = MPU6000_SPI_BUS,
    .CLKPolarity = SPI_POLARITY_HIGH,
    .CLKPhase = SPI_PHASE_2EDGE,
    .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8,
};

/* ICM20602 Instance */
static SPI_HandleTypeDef ICM20602_Bus_Instance;
static BspSPI_NorModeConfig_TypeDef ICM20602_BusCfg = {
    .Instance = ICM20602_SPI_BUS,
    .CLKPolarity = SPI_POLARITY_HIGH,
    .CLKPhase = SPI_PHASE_2EDGE,
    .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8,
};

static DevMPU6000Obj_TypeDef MPU6000Obj;
static DevICM20602Obj_TypeDef ICM20602Obj;

static Error_Obj_Typedef SrvIMU_ErrorList[] = {
    {
        .callback = NULL,
        .code = SrvIMU_PriCSPin_Init_Error,
        .desc = "Pri CS Pin Init Failed",
        .out = false,
        .proc_type = Error_Proc_Ignore,
    },
    {
        .callback = NULL,
        .code = SrvIMU_PriExtiPin_Init_Error,
        .desc = "Pri Ext Pin Init Failed",
        .out = false,
        .proc_type = Error_Proc_Ignore,
    },
    {
        .callback = NULL,
        .code = SrvIMU_PriBus_Init_Error,
        .desc = "Pri Bus Init Failed",
        .out = false,
        .proc_type = Error_Proc_Ignore,
    },
    {
        .callback = NULL,
        .code = SrvIMU_PriDev_Init_Error,
        .desc = "Pri Dev Init Failed",
        .out = false,
        .proc_type = Error_Proc_Ignore,
    },
    {
        .callback = NULL,
        .code = SrvIMU_SecCSPin_Init_Error,
        .desc = "Sec CS Pin Init Failed",
        .out = false,
        .proc_type = Error_Proc_Ignore,
    },
    {
        .callback = NULL,
        .code = SrvIMU_SecExtiPin_Init_Error,
        .desc = "Sec Ext Pin Init Failed",
        .out = false,
        .proc_type = Error_Proc_Ignore,
    },
    {
        .callback = NULL,
        .code = SrvIMU_SecBus_Init_Error,
        .desc = "Sec Bus Init Failed",
        .out = false,
        .proc_type = Error_Proc_Ignore,
    },
    {
        .callback = NULL,
        .code = SrvIMU_SecDev_Init_Error,
        .desc = "Sec Dev Init Failed",
        .out = false,
        .proc_type = Error_Proc_Ignore,
    },
    {
        .callback = NULL,
        .code = SrvIMU_SecDev_Init_Error,
        .desc = "Sec Dev Init Failed",
        .out = false,
        .proc_type = Error_Proc_Ignore,
    },
    {
        .callback = NULL,
        .code = SrvIMU_SecDev_Init_Error,
        .desc = "Sec Dev Init Failed",
        .item = NULL,
        .out = false,
        .proc_type = Error_Proc_Ignore,
    },
    {
        .callback = NULL,
        .code = SrvIMU_Sample_Init_Error,
        .desc = "Sec Sample Init Failed",
        .out = false,
        .proc_type = Error_Proc_Ignore,
    },
    {
        .callback = NULL,
        .code = SrvIMU_Sample_OverRange,
        .desc = "Sec Sample OverRange",
        .out = false,
        .proc_type = Error_Proc_Ignore,
    },
    {
        .callback = NULL,
        .code = SrvIMU_Sample_Blunt,
        .desc = "Sec Sample Blunt",
        .out = false,
        .proc_type = Error_Proc_Ignore,
    },
};

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
    /* create error log handle */
    SrvMPU_Error_Handle = ErrorTree_Create("SrvIMU_Error");

    /* regist all error to the error tree */
    Error_Register(SrvMPU_Error_Handle, SrvIMU_ErrorList, sizeof(SrvIMU_ErrorList));

    SrvIMU_ErrorCode_List PriIMU_Init_State = SrvIMU_PriIMU_Init();
    SrvIMU_ErrorCode_List SecIMU_Init_State = SrvIMU_SecIMU_Init();

    SrvMpu_Reg.PriDev_Init_State = false;
    SrvMpu_Reg.SecDev_Init_State = false;

    if (PriIMU_Init_State == SrvIMU_No_Error)
    {
        SrvMpu_Reg.PriDev_Init_State = true;
    }

    if (SecIMU_Init_State == SrvIMU_No_Error)
    {
        SrvMpu_Reg.SecDev_Init_State = true;
    }

    if (!SrvMpu_Reg.PriDev_Init_State && !SrvMpu_Reg.SecDev_Init_State)
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
                      MPU6000_SampleRate_4K,
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
    /* primary IMU Pin & Bus Init */
    if (!BspGPIO.out_init(ICM20602_CSPin))
        return SrvIMU_SecCSPin_Init_Error;

    if (!BspGPIO.exti_init(ICM20602_INTPin, SrvIMU_SecIMU_ExtiCallback))
        return SrvIMU_SecExtiPin_Init_Error;

    ICM20602_BusCfg.Pin = ICM20602_BusPin;

    DevICM20602.pre_init(&ICM20602Obj,
                         SrvIMU_SecIMU_CS_Ctl,
                         SrvIMU_SecIMU_BusTrans_Rec,
                         Runtime_DelayMs,
                         Get_CurrentRunningUs);

    DevICM20602.config(&ICM20602Obj,
                       ICM20602_SampleRate_4K,
                       ICM20602_Acc_16G,
                       ICM20602_Gyr_2000DPS);

    if (!BspSPI.init(ICM20602_BusCfg, &ICM20602_Bus_Instance))
        return SrvIMU_SecBus_Init_Error;

    if (!DevICM20602.init(&ICM20602Obj))
        return SrvIMU_SecDev_Init_Error;

    /* Set SPI Speed 20M */
    ICM20602_BusCfg.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;

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
    if (SrvMpu_Reg.PriDev_Init_State)
        DevMPU6000.set_drdy(&MPU6000Obj);
}

static void SrvIMU_SecIMU_ExtiCallback(void)
{
    if (SrvMpu_Reg.SecDev_Init_State)
        DevICM20602.set_ready(&ICM20602Obj);
}

int8_t SrvIMU_GetPri_InitError(void)
{
    return DevMPU6000.get_error(&MPU6000Obj);
}

int8_t SrvIMU_GetSec_InitError(void)
{
    return DevICM20602.get_error(&ICM20602Obj);
}
