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
static void SrvIMU_PriDev_InitError(uint8_t *p_arg, uint16_t size);
static void SrvIMU_SecDev_InitError(uint8_t *p_arg, uint16_t size);
static void SrvIMU_AllModule_InitError(uint8_t *p_arg, uint16_t size);
static void SrvIMU_PriSample_Undrdy(uint8_t *p_arg, uint16_t size);
static void SrvIMU_SecSample_Undrdy(uint8_t *p_arg, uint16_t size);

static SrvMpu_InitReg_TypeDef SrvMpu_Reg = {.PriDev_Init_State = false, .SecDev_Init_State = false};
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
        .callback = SrvIMU_PriDev_InitError,
        .code = SrvIMU_PriDev_Init_Error,
        .desc = "Pri Dev Init Failed",
        .out = false,
        .proc_type = Error_Proc_Immd,
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
        .callback = SrvIMU_SecDev_InitError,
        .code = SrvIMU_SecDev_Init_Error,
        .desc = "Sec Dev Init Failed",
        .out = true,
        .proc_type = Error_Proc_Immd,
    },
    {
        .callback = SrvIMU_AllModule_InitError,
        .code = SrvIMU_AllModule_Init_Error,
        .desc = "All IMU Module Init Failed",
        .out = true,
        .proc_type = Error_Proc_Immd,
    },
    {
        .callback = NULL,
        .code = SrvIMU_PriSample_Init_Error,
        .desc = "Pri Sample Init Failed",
        .out = false,
        .proc_type = Error_Proc_Ignore,
    },
    {
        .callback = NULL,
        .code = SrvIMU_PriSample_OverRange,
        .desc = "Pri Sample OverRange",
        .out = false,
        .proc_type = Error_Proc_Ignore,
    },
    {
        .callback = NULL,
        .code = SrvIMU_PriSample_Blunt,
        .desc = "Pri Sample Blunt",
        .out = false,
        .proc_type = Error_Proc_Ignore,
    },
    {
        .callback = SrvIMU_PriSample_Undrdy,
        .code = SrvIMU_PriSample_UnReady,
        .desc = "Pri Sample Unready",
        .out = false,
        .proc_type = Error_Proc_Immd,
    },
    {
        .callback = NULL,
        .code = SrvIMU_SecSample_Init_Error,
        .desc = "Sec Sample Init Failed",
        .out = false,
        .proc_type = Error_Proc_Ignore,
    },
    {
        .callback = NULL,
        .code = SrvIMU_SecSample_OverRange,
        .desc = "Sec Sample OverRange",
        .out = false,
        .proc_type = Error_Proc_Ignore,
    },
    {
        .callback = NULL,
        .code = SrvIMU_SecSample_Blunt,
        .desc = "Sec Sample Blunt",
        .out = false,
        .proc_type = Error_Proc_Ignore,
    },
    {
        .callback = SrvIMU_SecSample_Undrdy,
        .code = SrvIMU_SecSample_UnReady,
        .desc = "Sec Sample Unready",
        .out = false,
        .proc_type = Error_Proc_Immd,
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
    Error_Register(SrvMPU_Error_Handle, SrvIMU_ErrorList, sizeof(SrvIMU_ErrorList) / sizeof(SrvIMU_ErrorList[0]));

    SrvIMU_ErrorCode_List PriIMU_Init_State = SrvIMU_PriIMU_Init();
    SrvIMU_ErrorCode_List SecIMU_Init_State = SrvIMU_SecIMU_Init();

    SrvMpu_Reg.PriDev_Init_State = false;
    SrvMpu_Reg.SecDev_Init_State = false;

    if (PriIMU_Init_State == SrvIMU_No_Error)
    {
        SrvMpu_Reg.PriDev_Init_State = true;
    }
    else
        Error_Trigger(SrvMPU_Error_Handle, SrvIMU_PriDev_Init_Error, NULL, 0);

    if (SecIMU_Init_State == SrvIMU_No_Error)
    {
        SrvMpu_Reg.SecDev_Init_State = true;
    }
    else
        Error_Trigger(SrvMPU_Error_Handle, SrvIMU_SecDev_Init_Error, NULL, 0);

    if (!SrvMpu_Reg.PriDev_Init_State && !SrvMpu_Reg.SecDev_Init_State)
    {
        return SrvIMU_AllModule_Init_Error;
    }

    Error_Trigger(SrvMPU_Error_Handle, SrvIMU_AllModule_Init_Error, NULL, 0);

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

int8_t SrvIMU_GetPri_InitError(void)
{
    return DevMPU6000.get_error(&MPU6000Obj);
}

int8_t SrvIMU_GetSec_InitError(void)
{
    return DevICM20602.get_error(&ICM20602Obj);
}

static void SrvIMU_PriSample(void)
{
    /* pri imu init successed */
    if (SrvMpu_Reg.PriDev_Init_State)
    {
        /* pri imu module data ready triggered */
        if (DevMPU6000.get_drdy(&MPU6000Obj))
        {
        }
        else
            Error_Trigger(SrvMPU_Error_Handle, SrvIMU_PriSample_UnReady, NULL, 0);
    }
    else
        Error_Trigger(SrvMPU_Error_Handle, SrvIMU_PriSample_Init_Error, NULL, 0);
}

static void SrvIMU_SecSample(void)
{
    /* sec imu init successed */
    if (SrvMpu_Reg.SecDev_Init_State)
    {
        /* sec imu module data ready triggered */
        if (DevICM20602.get_ready(&ICM20602Obj))
        {
        }
        else
            Error_Trigger(SrvMPU_Error_Handle, SrvIMU_SecSample_UnReady, NULL, 0);
    }
    else
        Error_Trigger(SrvMPU_Error_Handle, SrvIMU_SecSample_Init_Error, NULL, 0);
}

/************************************************************ DataReady Pin Exti Callback *****************************************************************************/
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

/*************************************************************** Error Process Callback *******************************************************************************/
static void SrvIMU_PriDev_InitError(uint8_t *p_arg, uint16_t size)
{
    static uint8_t a;

    a++;
}

static void SrvIMU_SecDev_InitError(uint8_t *p_arg, uint16_t size)
{
    static uint8_t a;

    a++;
}

static void SrvIMU_AllModule_InitError(uint8_t *p_arg, uint16_t size)
{
    static uint8_t a;

    a++;
}

static void SrvIMU_PriSample_Undrdy(uint8_t *p_arg, uint16_t size)
{
    static uint8_t a;

    a++;
}

static void SrvIMU_SecSample_Undrdy(uint8_t *p_arg, uint16_t size)
{
    static uint8_t a;

    a++;
}
