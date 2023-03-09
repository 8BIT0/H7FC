#include "Srv_IMUSample.h"
#include "Dev_MPU6000.h"
#include "Dev_ICM20602.h"
#include "IO_Definition.h"
#include "runtime.h"
#include "debug_util.h"
#include "Bsp_SPI.h"
#include "error_log.h"
#include "Dev_Led.h"
#include <math.h>

/* use ENU coordinate */

#define IMU_Commu_TimeOut 1000
#define MPU_MODULE_INIT_RETRY 10 // init retry count 10

/* test var */
static uint32_t SrvIMU_PriIMU_Init_Error_CNT = 0;
static uint32_t SrvIMU_SecIMU_Init_Error_CNT = 0;
static uint32_t SrvIMU_ALLModule_Init_Error_CNT = 0;

/*
 *   PriIMU -> MPU6000
 *   SecIMU -> ICM20602
 */
static SrvMpu_Reg_TypeDef SrvMpu_Init_Reg;
static SrvMpu_Reg_TypeDef SrvMpu_Update_Reg;
static SrvIMU_Data_TypeDef PriIMU_Data;
static SrvIMU_Data_TypeDef SecIMU_Data;
static SrvIMU_Data_TypeDef PriIMU_Data_Lst;
static SrvIMU_Data_TypeDef SecIMU_Data_Lst;
static Error_Handler SrvMPU_Error_Handle = NULL;

/* internal variable */
/* MPU6000 Instance */
static SPI_HandleTypeDef MPU6000_Bus_Instance;
static BspSPI_NorModeConfig_TypeDef MPU6000_BusCfg = {
    .Instance = MPU6000_SPI_BUS,
    .CLKPolarity = SPI_POLARITY_HIGH,
    .CLKPhase = SPI_PHASE_2EDGE,
    .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4,
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

/************************************************************************ Error Tree Item ************************************************************************/
static void SrvIMU_PriDev_InitError(int16_t code, uint8_t *p_arg, uint16_t size);
static void SrvIMU_SecDev_InitError(int16_t code, uint8_t *p_arg, uint16_t size);
static void SrvIMU_AllModule_InitError(int16_t code, uint8_t *p_arg, uint16_t size);
static void SrvIMU_PriSample_Undrdy(uint8_t *p_arg, uint16_t size);
static void SrvIMU_SecSample_Undrdy(uint8_t *p_arg, uint16_t size);

static Error_Obj_Typedef SrvIMU_ErrorList[] = {
    {
        .out = false,
        .log = false,
        .prc_callback = NULL,
        .code = SrvIMU_PriCSPin_Init_Error,
        .desc = "Pri CS Pin Init Failed\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = false,
        .log = false,
        .prc_callback = NULL,
        .code = SrvIMU_PriExtiPin_Init_Error,
        .desc = "Pri Ext Pin Init Failed\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = false,
        .log = false,
        .prc_callback = NULL,
        .code = SrvIMU_PriBus_Init_Error,
        .desc = "Pri Bus Init Failed\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = true,
        .prc_callback = SrvIMU_PriDev_InitError,
        .code = SrvIMU_PriDev_Init_Error,
        .desc = "Pri Dev Init Failed\r\n",
        .proc_type = Error_Proc_Immd,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = false,
        .log = false,
        .prc_callback = NULL,
        .code = SrvIMU_SecCSPin_Init_Error,
        .desc = "Sec CS Pin Init Failed\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = false,
        .prc_callback = NULL,
        .code = SrvIMU_SecExtiPin_Init_Error,
        .desc = "Sec Ext Pin Init Failed\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = false,
        .log = false,
        .prc_callback = NULL,
        .code = SrvIMU_SecBus_Init_Error,
        .desc = "Sec Bus Init Failed\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = SrvIMU_SecDev_InitError,
        .code = SrvIMU_SecDev_Init_Error,
        .desc = "Sec Dev Init Failed\r\n",
        .proc_type = Error_Proc_Immd,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = SrvIMU_AllModule_InitError,
        .code = SrvIMU_AllModule_Init_Error,
        .desc = "All IMU Module Init Failed\r\n",
        .proc_type = Error_Proc_Immd,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
};
/************************************************************************ Error Tree Item ************************************************************************/

/* external function */
static SrvIMU_ErrorCode_List SrvIMU_Init(void);
static bool SrvIMU_Sample(void);
static SrvIMU_Data_TypeDef SrvIMU_Get_Data(SrvIMU_Module_Type type);
static void SrvIMU_ErrorProc(void);

/* internal function */
static int8_t SrvIMU_PriIMU_Init(void);
static int8_t SrvIMU_SecIMU_Init(void);
static void SrvIMU_PriIMU_ExtiCallback(void);
static void SrvIMU_SecIMU_ExtiCallback(void);
static void SrvIMU_PriIMU_CS_Ctl(bool state);
static void SrvIMU_SecIMU_CS_Ctl(bool state);
static bool SrvIMU_PriIMU_BusTrans_Rec(uint8_t *Tx, uint8_t *Rx, uint16_t size);
static bool SrvIMU_SecIMU_BusTrans_Rec(uint8_t *Tx, uint8_t *Rx, uint16_t size);

SrvIMU_TypeDef SrvIMU = {
    .init = SrvIMU_Init,
    .sample = SrvIMU_Sample,
    .get_data = SrvIMU_Get_Data,
    .error_proc = SrvIMU_ErrorProc,
};

static SrvIMU_ErrorCode_List SrvIMU_Init(void)
{
    /* create error log handle */
    SrvMPU_Error_Handle = ErrorLog.create("SrvIMU_Error");

    /* regist all error to the error tree */
    ErrorLog.registe(SrvMPU_Error_Handle, SrvIMU_ErrorList, sizeof(SrvIMU_ErrorList) / sizeof(SrvIMU_ErrorList[0]));

    SrvIMU_ErrorCode_List PriIMU_Init_State = SrvIMU_PriIMU_Init();
    SrvIMU_ErrorCode_List SecIMU_Init_State = SrvIMU_SecIMU_Init();

    SrvMpu_Init_Reg.val = 0;
    SrvMpu_Update_Reg.val = 0;

    if (PriIMU_Init_State == SrvIMU_No_Error)
    {
        SrvMpu_Init_Reg.sec.Pri_State = true;
    }
    else
        ErrorLog.trigger(SrvMPU_Error_Handle, SrvIMU_PriDev_Init_Error, &SrvIMU_PriIMU_Init_Error_CNT, sizeof(SrvIMU_PriIMU_Init_Error_CNT));

    if (SecIMU_Init_State == SrvIMU_No_Error)
    {
        SrvMpu_Init_Reg.sec.Sec_State = true;
    }
    else
        ErrorLog.trigger(SrvMPU_Error_Handle, SrvIMU_SecDev_Init_Error, &SrvIMU_SecIMU_Init_Error_CNT, sizeof(SrvIMU_SecIMU_Init_Error_CNT));

    memset(&PriIMU_Data, NULL, sizeof(PriIMU_Data));
    memset(&SecIMU_Data, NULL, sizeof(SecIMU_Data));

    memset(&PriIMU_Data_Lst, NULL, sizeof(PriIMU_Data_Lst));
    memset(&SecIMU_Data_Lst, NULL, sizeof(SecIMU_Data_Lst));

    if (!SrvMpu_Init_Reg.sec.Pri_State && !SrvMpu_Init_Reg.sec.Sec_State)
    {
        ErrorLog.trigger(SrvMPU_Error_Handle, SrvIMU_AllModule_Init_Error, &SrvIMU_ALLModule_Init_Error_CNT, sizeof(SrvIMU_ALLModule_Init_Error_CNT));
        return SrvIMU_AllModule_Init_Error;
    }
    else if (!SrvMpu_Init_Reg.sec.Pri_State && SrvMpu_Init_Reg.sec.Sec_State)
    {
        return SrvIMU_PriDev_Init_Error;
    }
    else if (SrvMpu_Init_Reg.sec.Pri_State && !SrvMpu_Init_Reg.sec.Sec_State)
    {
        return SrvIMU_SecDev_Init_Error;
    }
    else
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

/************************************************************ Module Sample API Function *****************************************************************************/
static SrvIMU_SampleErrorCode_List SrvIMU_DataCheck(IMUData_TypeDef *data, uint8_t acc_range, uint16_t gyr_range)
{
    float Acc_Range_Max = acc_range * MPU_RANGE_MAX_THRESHOLD;
    float Gyr_Range_Max = gyr_range * MPU_RANGE_MAX_THRESHOLD;
    float Acc_Range_Min = acc_range * MPU_RANGE_MIN_THRESHOLD;
    float Gyr_Range_Min = gyr_range * MPU_RANGE_MIN_THRESHOLD;

    for (uint8_t axis = Axis_X; axis < Axis_Sum; axis++)
    {
        /* over range chack */
        {
            /* check acc data range */
            if (fabs(data->acc_flt[axis]) > Acc_Range_Max)
                return SrvIMU_Sample_Data_Acc_OverRange;

            /* check gyr data range */
            if (fabs(data->gyr_flt[axis]) > Gyr_Range_Max)
                return SrvIMU_Sample_Data_Gyr_OverRange;
        }

        /* blunt data check */
        {
            if (data->acc_int_lst[axis] != 0)
            {
                if ((fabs(data->acc_flt[axis]) <= Acc_Range_Min) && (data->acc_int[axis] == data->acc_int_lst[axis]))
                {
                    data->acc_blunt_cnt[axis]++;

                    if (data->acc_blunt_cnt >= IMU_BLUNT_SAMPLE_CNT)
                    {
                        data->acc_blunt_cnt[axis] = 0;
                        return SrvIMU_Sample_Data_Acc_Blunt;
                    }
                }
                else
                    data->acc_blunt_cnt[axis] = 0;
            }

            if (data->gyr_int_lst[axis] != 0)
            {
                if ((fabs(data->gyr_flt[axis]) <= Gyr_Range_Min) && (data->gyr_int[axis] == data->gyr_int_lst[axis]))
                {
                    data->gyr_blunt_cnt[axis]++;

                    if (data->gyr_blunt_cnt[axis] >= IMU_BLUNT_SAMPLE_CNT)
                    {
                        data->gyr_blunt_cnt[axis] = 0;
                        return SrvIMU_Sample_Data_Gyr_Blunt;
                    }
                }
                else
                    data->gyr_blunt_cnt[axis] = 0;
            }
        }
    }

    return SrvIMU_Sample_NoError;
}

static bool SrvIMU_Sample(void)
{
    static SYSTEM_RunTime PriSample_Rt_Lst = 0;
    static SYSTEM_RunTime SecSample_Rt_Lst = 0;
    uint8_t i = Axis_X;
    bool pri_sample_state = true;
    bool sec_sample_state = true;
    IMUModuleScale_TypeDef pri_imu_scale;
    IMUModuleScale_TypeDef sec_imu_scale;

    /* don`t use error tree down below it may decrease code efficient */
    /* trigger error directly */

    /* pri imu init successed */
    if (SrvMpu_Init_Reg.sec.Pri_State)
    {
        pri_imu_scale = DevMPU6000.get_scale(MPU6000Obj);
        PriIMU_Data.acc_scale = pri_imu_scale.acc_scale;
        PriIMU_Data.gyr_scale = pri_imu_scale.gyr_scale;

        /* pri imu module data ready triggered */
        if (DevMPU6000.get_drdy(&MPU6000Obj) && DevMPU6000.sample(&MPU6000Obj))
        {
            /* lock */
            SrvMpu_Update_Reg.sec.Pri_State = true;

            PriIMU_Data.cycle_cnt++;
            PriIMU_Data.time_stamp = MPU6000Obj.OriData.time_stamp;

            /* check Primary IMU module Sample is correct or not */
            if (PriSample_Rt_Lst)
            {
                if (PriIMU_Data.time_stamp <= PriSample_Rt_Lst)
                    pri_sample_state = false;
            }

            if (pri_sample_state)
            {
                /* update pri imu data */
                PriIMU_Data.tempera = MPU6000Obj.OriData.temp_flt;

                /* Pri imu data validation check */
                PriIMU_Data.error_code = SrvIMU_DataCheck(&MPU6000Obj.OriData, MPU6000Obj.AccTrip, MPU6000Obj.GyrTrip);

                for (i = Axis_X; i < Axis_Sum; i++)
                {
                    PriIMU_Data.acc[i] = MPU6000Obj.OriData.acc_flt[i];
                    PriIMU_Data.gyr[i] = MPU6000Obj.OriData.gyr_flt[i];

                    /* update last time value */
                    if ((PriIMU_Data.error_code != SrvIMU_Sample_Data_Acc_OverRange) &&
                        (PriIMU_Data.error_code != SrvIMU_Sample_Data_Gyr_OverRange))
                    {
                        MPU6000Obj.OriData.acc_int_lst[i] = MPU6000Obj.OriData.acc_int[i];
                        MPU6000Obj.OriData.gyr_int_lst[i] = MPU6000Obj.OriData.gyr_int[i];
                    }
                };

                /* filter Pri IMU Module data */

                /* unlock */
                SrvMpu_Update_Reg.sec.Pri_State = false;
                PriIMU_Data_Lst = PriIMU_Data;
            }
        }
        else
        {
            SrvIMU_PriSample_Undrdy(NULL, 0);
            pri_sample_state = false;
            PriIMU_Data_Lst.error_code = SrvIMU_Sample_Module_UnReady;
        }

        PriSample_Rt_Lst = PriIMU_Data.time_stamp;
    }
    else
        pri_sample_state = false;

    /* sec imu init successed */
    if (SrvMpu_Init_Reg.sec.Sec_State)
    {
        sec_imu_scale = DevICM20602.get_scale(ICM20602Obj);
        SecIMU_Data.acc_scale = sec_imu_scale.acc_scale;
        SecIMU_Data.gyr_scale = sec_imu_scale.gyr_scale;

        /* sec imu module data ready triggered */
        if (DevICM20602.get_ready(&ICM20602Obj) && DevICM20602.sample(&ICM20602Obj))
        {
            /* lock */
            SrvMpu_Update_Reg.sec.Sec_State = true;

            SecIMU_Data.cycle_cnt++;
            SecIMU_Data.time_stamp = ICM20602Obj.OriData.time_stamp;

            /* check Secondry IMU module Sample is correct or not */
            if (SecSample_Rt_Lst)
            {
                if (SecIMU_Data.time_stamp <= SecSample_Rt_Lst)
                    sec_sample_state = false;
            }

            if (sec_sample_state)
            {
                /* update sec imu data */
                SecIMU_Data.tempera = ICM20602Obj.OriData.temp_flt;

                /* Sec imu data validation check */
                SecIMU_Data.error_code = SrvIMU_DataCheck(&ICM20602Obj.OriData, ICM20602Obj.AccTrip, ICM20602Obj.GyrTrip);

                for (i = Axis_X; i < Axis_Sum; i++)
                {
                    SecIMU_Data.acc[i] = ICM20602Obj.OriData.acc_flt[i];
                    SecIMU_Data.gyr[i] = ICM20602Obj.OriData.gyr_flt[i];

                    /* update Sec last value */
                    if ((SecIMU_Data.error_code != SrvIMU_Sample_Data_Acc_OverRange) &&
                        (SecIMU_Data.error_code != SrvIMU_Sample_Data_Gyr_OverRange))
                    {
                        ICM20602Obj.OriData.acc_int_lst[i] = ICM20602Obj.OriData.acc_int[i];
                        ICM20602Obj.OriData.gyr_int_lst[i] = ICM20602Obj.OriData.gyr_int[i];
                    }
                }

                /* filter Sec IMU Module data */

                /* unlock */
                SrvMpu_Update_Reg.sec.Sec_State = false;
                SecIMU_Data_Lst = SecIMU_Data;
            }
        }
        else
        {
            SrvIMU_SecSample_Undrdy(NULL, 0);
            sec_sample_state = false;
            SecIMU_Data.error_code = SrvIMU_Sample_Module_UnReady;
        }

        SecSample_Rt_Lst = SecIMU_Data.time_stamp;
    }
    else
        sec_sample_state = false;

    return (pri_sample_state | sec_sample_state);
}

static bool SrvIMU_Calibration(void)
{
}

static SrvIMU_Data_TypeDef SrvIMU_Get_Data(SrvIMU_Module_Type type)
{
    SrvIMU_Data_TypeDef imu_data_tmp;

    memset(&imu_data_tmp, NULL, sizeof(SrvIMU_Data_TypeDef));

    if (type == SrvIMU_PriModule)
    {
        if (!SrvMpu_Update_Reg.sec.Pri_State)
        {
            memcpy(&imu_data_tmp, &PriIMU_Data, sizeof(SrvIMU_Data_TypeDef));
        }
        else
            memcpy(&imu_data_tmp, &PriIMU_Data_Lst, sizeof(SrvIMU_Data_TypeDef));
    }
    else if (type == SrvIMU_SecModule)
    {
        if (!SrvMpu_Update_Reg.sec.Sec_State)
        {
            memcpy(&imu_data_tmp, &SecIMU_Data, sizeof(SrvIMU_Data_TypeDef));
        }
        else
            memcpy(&imu_data_tmp, &SecIMU_Data_Lst, sizeof(SrvIMU_Data_TypeDef));
    }

    return imu_data_tmp;
}

static void SrvIMU_ErrorProc(void)
{
    ErrorLog.proc(SrvMPU_Error_Handle);
}

/************************************************************ DataReady Pin Exti Callback *****************************************************************************/
static void SrvIMU_PriIMU_ExtiCallback(void)
{
    if (SrvMpu_Init_Reg.sec.Pri_State)
        DevMPU6000.set_drdy(&MPU6000Obj);
}

static void SrvIMU_SecIMU_ExtiCallback(void)
{
    if (SrvMpu_Init_Reg.sec.Sec_State)
        DevICM20602.set_ready(&ICM20602Obj);
}

/*************************************************************** Error Process Callback *******************************************************************************/
static void SrvIMU_PriDev_InitError(int16_t code, uint8_t *p_arg, uint16_t size)
{
    (*(uint32_t *)p_arg)++;
}

static void SrvIMU_SecDev_InitError(int16_t code, uint8_t *p_arg, uint16_t size)
{
    (*(uint32_t *)p_arg)++;
}

static void SrvIMU_AllModule_InitError(int16_t code, uint8_t *p_arg, uint16_t size)
{
    (*(uint32_t *)p_arg)++;
}

static void SrvIMU_PriSample_Undrdy(uint8_t *p_arg, uint16_t size)
{
}

static void SrvIMU_SecSample_Undrdy(uint8_t *p_arg, uint16_t size)
{
}
