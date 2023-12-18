/*
 * CODE: 8_B!T0
 */
#include "Srv_IMUSample.h"
#include "Dev_MPU6000.h"
#include "Dev_ICM20602.h"
#include "Dev_ICM426xx.h"
#include "IO_Definition.h"
#include "Srv_OsCommon.h"
#include "debug_util.h"
#include "Bsp_SPI.h"
#include "error_log.h"
#include "Dev_Led.h"
#include "../Algorithm/Filter_Dep/filter.h"
#include <math.h>

/* use ENU coordinate */
/* IMU coordinate is x->forward y->right z->down */

/*
 * Angular Speed Over Speed Threshold
 * Angular Speed Per Millscond
 */
#define ANGULAR_ACCECLERATION_THRESHOLD 10 / 1.0f // angular speed accelerate from 0 to 100 deg/s in 1 Ms

typedef struct
{
    SrvIMU_SensorID_List type;
    void *obj_ptr;
    IMUData_TypeDef *OriData_ptr;

    uint8_t acc_trip;
    uint16_t gyr_trip;

    bool (*get_drdy)(void *obj);
    bool (*sample)(void *obj);
    IMUModuleScale_TypeDef (*get_scale)(void *obj);
    void (*set_drdy)(void *obj);
    IMU_Error_TypeDef (*get_error)(void *obj);
}SrvIMU_InuseSensorObj_TypeDef;

/* test var */
static uint32_t SrvIMU_PriIMU_Init_Error_CNT = 0;
static uint32_t SrvIMU_SecIMU_Init_Error_CNT = 0;
static uint32_t SrvIMU_ALLModule_Init_Error_CNT = 0;
static uint32_t SrvIMU_Reupdate_Statistics_CNT = 0;

/* PriIMU Butterworth filter object handle */
static BWF_Object_Handle PriIMU_Gyr_LPF_Handle[Axis_Sum] = {0};
static BWF_Object_Handle PriIMU_Acc_LPF_Handle[Axis_Sum] = {0};

/* SecIMU Butterworth filter object handle */
static BWF_Object_Handle SecIMU_Gyr_LPF_Handle[Axis_Sum] = {0};
static BWF_Object_Handle SecIMU_Acc_LPF_Handle[Axis_Sum] = {0};

/* Gyro Calibration Monitor */
static SrvIMU_CalibMonitor_TypeDef Gyro_Calib_Monitor;

/* Gyro Calibration Zero Offset Val */
static float PriIMU_Gyr_ZeroOffset[Axis_Sum] = {0.0f};
static float SecIMU_Gyr_ZeroOffset[Axis_Sum] = {0.0f};

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
static SrvIMU_Data_TypeDef IMU_Data;
static SrvIMU_Data_TypeDef IMU_Data_Lst;
static Error_Handler SrvMPU_Error_Handle = NULL;

/* internal variable */
/* MPU6000 Instance */
static SPI_HandleTypeDef PriIMU_Bus_Instance;
static BspSPI_NorModeConfig_TypeDef PriIMU_BusCfg = {
    .Instance = PriIMU_SPI_BUS,
    .CLKPolarity = SPI_POLARITY_HIGH,
    .CLKPhase = SPI_PHASE_2EDGE,
    .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4,
};

/* ICM20602 Instance */
static SPI_HandleTypeDef SecIMU_Bus_Instance;
static BspSPI_NorModeConfig_TypeDef SecIMU_BusCfg = {
    .Instance = SecIMU_SPI_BUS,
    .CLKPolarity = SPI_POLARITY_LOW,
    .CLKPhase = SPI_PHASE_1EDGE,
    .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2,
};

static DevMPU6000Obj_TypeDef MPU6000Obj;
static DevICM20602Obj_TypeDef ICM20602Obj;
static DevICM426xxObj_TypeDef ICM42688PObj;
static DevICM426xxObj_TypeDef ICM42605Obj;

static SrvIMU_InuseSensorObj_TypeDef InUse_PriIMU_Obj;
static SrvIMU_InuseSensorObj_TypeDef InUse_SecIMU_Obj;

/************************************************************************ Error Tree Item ************************************************************************/
static void SrvIMU_PriDev_Filter_InitError(int16_t code, uint8_t *p_arg, uint16_t size);
static void SrvIMU_SecDev_Filter_InitError(int16_t code, uint8_t *p_arg, uint16_t size);
static void SrvIMU_Dev_InitError(int16_t code, uint8_t *p_arg, uint16_t size);
static void SrvIMU_AllModule_InitError(int16_t code, uint8_t *p_arg, uint16_t size);
static void SrvIMU_PriSample_Undrdy(uint8_t *p_arg, uint16_t size);
static void SrvIMU_SecSample_Undrdy(uint8_t *p_arg, uint16_t size);

static Error_Obj_Typedef SrvIMU_ErrorList[] = {
    {
        .out = true,
        .log = false,
        .prc_callback = NULL,
        .code = SrvIMU_SecDev_Detect_Error,
        .desc = "Sec IMU None Detected\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = NULL,
        .code = SrvIMU_PriDev_Detect_Error,
        .desc = "Pri IMU None Detected\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = SrvIMU_SecDev_Filter_InitError,
        .code = SrvIMU_SecIMU_Filter_Init_Error,
        .desc = "Sec IMU Filter Init Failed\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = SrvIMU_PriDev_Filter_InitError,
        .code = SrvIMU_PriIMU_Filter_Init_Error,
        .desc = "Pri IMU Filter Init Failed\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
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
        .out = true,
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
        .out = true,
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
        .log = false,
        .prc_callback = SrvIMU_Dev_InitError,
        .code = SrvIMU_PriDev_Init_Error,
        .desc = "Pri Dev Init Failed\r\n",
        .proc_type = Error_Proc_Immd,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
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
        .out = true,
        .log = false,
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
        .out = true,
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
        .prc_callback = SrvIMU_Dev_InitError,
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
static bool SrvIMU_Sample(SrvIMU_SampleMode_List mode);
static bool SrvIMU_Get_Data(SrvIMU_Module_Type type, SrvIMU_Data_TypeDef *data);
static void SrvIMU_ErrorProc(void);
static float SrvIMU_Get_MaxAngularSpeed_Diff(void);
static GenCalib_State_TypeList SrvIMU_Calib_GyroZeroOffset(uint32_t calib_cycle, uint32_t *calib_cycle_cnt, float *pri_gyr, float *sec_gyr);
static GenCalib_State_TypeList SrvIMU_Set_Calib(uint32_t calb_cycle);
static GenCalib_State_TypeList SrvIMU_Get_Calib(void);
static bool SrvIMU_Get_Range(SrvIMU_Module_Type module, SrvIMU_Range_TypeDef *range);

/* internal function */
static int8_t SrvIMU_PriIMU_Init(void);
static int8_t SrvIMU_SecIMU_Init(void);
static void SrvIMU_PriIMU_ExtiCallback(void);
static void SrvIMU_SecIMU_ExtiCallback(void);
static void SrvIMU_PriIMU_CS_Ctl(bool state);
static void SrvIMU_SecIMU_CS_Ctl(bool state);
static bool SrvIMU_PriIMU_BusTrans_Rec(uint8_t *Tx, uint8_t *Rx, uint16_t size);
static bool SrvIMU_SecIMU_BusTrans_Rec(uint8_t *Tx, uint8_t *Rx, uint16_t size);
static bool SrvIMU_Detect_AngularOverSpeed(float angular_speed, float lst_angular_speed, float ms_diff);
static SrvIMU_SensorID_List SrvIMU_AutoDetect(bus_trans_callback trans, cs_ctl_callback cs_ctl);

SrvIMU_TypeDef SrvIMU = {
    .init = SrvIMU_Init,
    .sample = SrvIMU_Sample,
    .get_data = SrvIMU_Get_Data,
    .get_range = SrvIMU_Get_Range,
    .error_proc = SrvIMU_ErrorProc,
    .set_calib = SrvIMU_Set_Calib,
    .get_calib = SrvIMU_Get_Calib,
    .get_max_angular_speed_diff = SrvIMU_Get_MaxAngularSpeed_Diff,
};

static SrvIMU_ErrorCode_List SrvIMU_Init(void)
{
    FilterParam_Obj_TypeDef *Gyr_Filter_Ptr = NULL;
    FilterParam_Obj_TypeDef *Acc_Filter_Ptr = NULL;

    CREATE_FILTER_PARAM_OBJ(Gyr, 5, 30Hz, 1K, Gyr_Filter_Ptr);
    CREATE_FILTER_PARAM_OBJ(Acc, 5, 30Hz, 1K, Acc_Filter_Ptr);

    memset(&InUse_PriIMU_Obj, 0, sizeof(InUse_PriIMU_Obj));
    memset(&InUse_SecIMU_Obj, 0, sizeof(InUse_SecIMU_Obj));

    /* init gyro calibration monitor */
    Gyro_Calib_Monitor.state = Calib_Start;
    Gyro_Calib_Monitor.calib_cycle = GYR_STATIC_CALIB_CYCLE;
    Gyro_Calib_Monitor.cur_cycle = Gyro_Calib_Monitor.calib_cycle;

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

        /* init filter */
        PriIMU_Gyr_LPF_Handle[Axis_X] = Butterworth.init(Gyr_Filter_Ptr);
        PriIMU_Gyr_LPF_Handle[Axis_Y] = Butterworth.init(Gyr_Filter_Ptr);
        PriIMU_Gyr_LPF_Handle[Axis_Z] = Butterworth.init(Gyr_Filter_Ptr);

        PriIMU_Acc_LPF_Handle[Axis_X] = Butterworth.init(Acc_Filter_Ptr);
        PriIMU_Acc_LPF_Handle[Axis_Y] = Butterworth.init(Acc_Filter_Ptr);
        PriIMU_Acc_LPF_Handle[Axis_Z] = Butterworth.init(Acc_Filter_Ptr);

        for(uint8_t i = Axis_X; i < Axis_Sum; i++)
        {
            if( (PriIMU_Gyr_LPF_Handle[i] == 0) || 
                (PriIMU_Acc_LPF_Handle[i] == 0))
            {
                ErrorLog.trigger(SrvMPU_Error_Handle, SrvIMU_PriIMU_Filter_Init_Error, NULL, 0);
                return SrvIMU_PriIMU_Filter_Init_Error;
            }
        }
    }
    else
        ErrorLog.trigger(SrvMPU_Error_Handle, PriIMU_Init_State, &InUse_PriIMU_Obj, sizeof(InUse_PriIMU_Obj));

    if (SecIMU_Init_State == SrvIMU_No_Error)
    {
        SrvMpu_Init_Reg.sec.Sec_State = true;

        /* init filter */
        for(uint8_t i = Axis_X; i < Axis_Sum; i++)
        {
            SecIMU_Gyr_LPF_Handle[i] = Butterworth.init(Gyr_Filter_Ptr);
            SecIMU_Acc_LPF_Handle[i] = Butterworth.init(Acc_Filter_Ptr);

            if( (SecIMU_Gyr_LPF_Handle[i] == 0) || 
                (SecIMU_Acc_LPF_Handle[i] == 0))
            {
                ErrorLog.trigger(SrvMPU_Error_Handle, SrvIMU_SecIMU_Filter_Init_Error, NULL, 0);
                return SrvIMU_SecIMU_Filter_Init_Error;
            }
        }
    }
    else
        ErrorLog.trigger(SrvMPU_Error_Handle, SecIMU_Init_State, &InUse_SecIMU_Obj, sizeof(InUse_SecIMU_Obj));

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
    if (!BspGPIO.out_init(PriIMU_CSPin))
        return SrvIMU_PriCSPin_Init_Error;

    if (!BspGPIO.exti_init(PriIMU_INTPin, SrvIMU_PriIMU_ExtiCallback))
        return SrvIMU_PriExtiPin_Init_Error;

    PriIMU_BusCfg.Pin = PriIMU_BusPin;
    if (!BspSPI.init(PriIMU_BusCfg, &PriIMU_Bus_Instance))
        return SrvIMU_PriBus_Init_Error;

    switch(SrvIMU_AutoDetect(SrvIMU_PriIMU_BusTrans_Rec, SrvIMU_PriIMU_CS_Ctl))
    {
        case SrvIMU_Dev_MPU6000:
            InUse_PriIMU_Obj.type = SrvIMU_Dev_MPU6000;
            DevMPU6000.pre_init(&MPU6000Obj,
                                SrvIMU_PriIMU_CS_Ctl,
                                SrvIMU_PriIMU_BusTrans_Rec,
                                SrvOsCommon.delay_ms,
                                SrvOsCommon.get_os_ms);

            DevMPU6000.config(&MPU6000Obj,
                               MPU6000_SampleRate_2K,
                               MPU6000_Acc_16G,
                               MPU6000_Gyr_2000DPS);

            InUse_PriIMU_Obj.obj_ptr = &MPU6000Obj;
            InUse_PriIMU_Obj.OriData_ptr = &(MPU6000Obj.OriData);
            InUse_PriIMU_Obj.acc_trip = MPU6000Obj.PHY_AccTrip_Val;
            InUse_PriIMU_Obj.gyr_trip = MPU6000Obj.PHY_GyrTrip_Val;
            
            InUse_PriIMU_Obj.set_drdy = DevMPU6000.set_ready;
            InUse_PriIMU_Obj.get_drdy = DevMPU6000.get_ready;
            InUse_PriIMU_Obj.get_scale = DevMPU6000.get_scale;
            InUse_PriIMU_Obj.sample = DevMPU6000.sample;
            InUse_PriIMU_Obj.get_error = DevMPU6000.get_error;

            if (!DevMPU6000.init(&MPU6000Obj))
                return SrvIMU_PriDev_Init_Error;
        break;

        case SrvIMU_Dev_ICM20602:
            InUse_PriIMU_Obj.type = SrvIMU_Dev_ICM20602;
            DevICM20602.pre_init(&ICM20602Obj,
                                 SrvIMU_PriIMU_CS_Ctl,
                                 SrvIMU_PriIMU_BusTrans_Rec,
                                 SrvOsCommon.delay_ms,
                                 SrvOsCommon.get_os_ms);

            DevICM20602.config(&ICM20602Obj,
                                ICM20602_SampleRate_2K,
                                ICM20602_Acc_16G,
                                ICM20602_Gyr_2000DPS);

            InUse_PriIMU_Obj.obj_ptr = &ICM20602Obj;
            InUse_PriIMU_Obj.OriData_ptr = &(ICM20602Obj.OriData);
            InUse_PriIMU_Obj.acc_trip = ICM20602Obj.PHY_AccTrip_Val;
            InUse_PriIMU_Obj.gyr_trip = ICM20602Obj.PHY_GyrTrip_Val;

            InUse_PriIMU_Obj.set_drdy = DevICM20602.set_ready;
            InUse_PriIMU_Obj.get_drdy = DevICM20602.get_ready;
            InUse_PriIMU_Obj.get_scale = DevICM20602.get_scale;
            InUse_PriIMU_Obj.sample = DevICM20602.sample;
            InUse_PriIMU_Obj.get_error = DevICM20602.get_error;

            if (!DevICM20602.init(&ICM20602Obj))
                return SrvIMU_PriDev_Init_Error;
        break;

        case SrvIMU_Dev_ICM42688P:
            InUse_PriIMU_Obj.type = SrvIMU_Dev_ICM42688P;
            DevICM426xx.pre_init(&ICM42688PObj,
                                 SrvIMU_PriIMU_CS_Ctl,
                                 SrvIMU_PriIMU_BusTrans_Rec,
                                 SrvOsCommon.delay_ms,
                                 SrvOsCommon.get_os_ms);

            DevICM426xx.config(&ICM42688PObj,
                                ICM426xx_SampleRate_2K,
                                ICM426xx_Acc_16G,
                                ICM426xx_Gyr_2000DPS);

            InUse_PriIMU_Obj.obj_ptr = &ICM42688PObj;
            InUse_PriIMU_Obj.OriData_ptr = &(ICM42688PObj.OriData);
            InUse_PriIMU_Obj.acc_trip = ICM42688PObj.PHY_AccTrip_Val;
            InUse_PriIMU_Obj.gyr_trip = ICM42688PObj.PHY_GyrTrip_Val;

            InUse_PriIMU_Obj.set_drdy = DevICM426xx.set_ready;
            InUse_PriIMU_Obj.get_drdy = DevICM426xx.get_ready;
            InUse_PriIMU_Obj.get_scale = DevICM426xx.get_scale;
            InUse_PriIMU_Obj.sample = DevICM426xx.sample;
            InUse_PriIMU_Obj.get_error = DevICM426xx.get_error;

            if (!DevICM426xx.init(&ICM42688PObj))
                return SrvIMU_PriDev_Init_Error;
        break;

        case SrvIMU_Dev_ICM42605:
            InUse_PriIMU_Obj.type = SrvIMU_Dev_ICM42605;
            DevICM426xx.pre_init(&ICM42605Obj,
                                 SrvIMU_PriIMU_CS_Ctl,
                                 SrvIMU_PriIMU_BusTrans_Rec,
                                 SrvOsCommon.delay_ms,
                                 SrvOsCommon.get_os_ms);

            DevICM426xx.config(&ICM42605Obj,
                                ICM426xx_SampleRate_2K,
                                ICM426xx_Acc_16G,
                                ICM426xx_Gyr_2000DPS);

            InUse_PriIMU_Obj.obj_ptr = &ICM42605Obj;
            InUse_PriIMU_Obj.OriData_ptr = &(ICM42605Obj.OriData);
            InUse_PriIMU_Obj.acc_trip = ICM42605Obj.PHY_AccTrip_Val;
            InUse_PriIMU_Obj.gyr_trip = ICM42605Obj.PHY_GyrTrip_Val;

            InUse_PriIMU_Obj.set_drdy = DevICM426xx.set_ready;
            InUse_PriIMU_Obj.get_drdy = DevICM426xx.get_ready;
            InUse_PriIMU_Obj.get_scale = DevICM426xx.get_scale;
            InUse_PriIMU_Obj.sample = DevICM426xx.sample;
            InUse_PriIMU_Obj.get_error = DevICM426xx.get_error;

            if (!DevICM426xx.init(&ICM42605Obj))
                return SrvIMU_PriDev_Init_Error;
        break;

        default: return SrvIMU_PriDev_Detect_Error;
    }

    return SrvIMU_No_Error;
}

/* init primary IMU Device */
static SrvIMU_ErrorCode_List SrvIMU_SecIMU_Init(void)
{
    /* primary IMU Pin & Bus Init */
    if (!BspGPIO.out_init(SecIMU_CSPin))
        return SrvIMU_SecCSPin_Init_Error;

    if (!BspGPIO.exti_init(SecIMU_INTPin, SrvIMU_SecIMU_ExtiCallback))
        return SrvIMU_SecExtiPin_Init_Error;

    SecIMU_BusCfg.Pin = SecIMU_BusPin;
    if (!BspSPI.init(SecIMU_BusCfg, &SecIMU_Bus_Instance))
        return SrvIMU_SecBus_Init_Error;

    switch(SrvIMU_AutoDetect(SrvIMU_SecIMU_BusTrans_Rec, SrvIMU_SecIMU_CS_Ctl))
    {
        case SrvIMU_Dev_MPU6000:
            InUse_SecIMU_Obj.type = SrvIMU_Dev_MPU6000;
            DevMPU6000.pre_init(&MPU6000Obj,
                                SrvIMU_SecIMU_CS_Ctl,
                                SrvIMU_SecIMU_BusTrans_Rec,
                                SrvOsCommon.delay_ms,
                                SrvOsCommon.get_os_ms);

            DevMPU6000.config(&MPU6000Obj,
                               MPU6000_SampleRate_2K,
                               MPU6000_Acc_16G,
                               MPU6000_Gyr_2000DPS);

            InUse_SecIMU_Obj.obj_ptr = &MPU6000Obj;
            InUse_SecIMU_Obj.OriData_ptr = &(MPU6000Obj.OriData);
            InUse_SecIMU_Obj.acc_trip = MPU6000Obj.PHY_AccTrip_Val;
            InUse_SecIMU_Obj.gyr_trip = MPU6000Obj.PHY_GyrTrip_Val;

            InUse_SecIMU_Obj.set_drdy = DevMPU6000.set_ready;
            InUse_SecIMU_Obj.get_drdy = DevMPU6000.get_ready;
            InUse_SecIMU_Obj.get_scale = DevMPU6000.get_scale;
            InUse_SecIMU_Obj.sample = DevMPU6000.sample;
            InUse_SecIMU_Obj.get_error = DevMPU6000.get_error;

            if (!DevMPU6000.init(&MPU6000Obj))
                return SrvIMU_SecDev_Init_Error;
        break;

        case SrvIMU_Dev_ICM20602:
            InUse_SecIMU_Obj.type = SrvIMU_Dev_ICM20602;
            DevICM20602.pre_init(&ICM20602Obj,
                                 SrvIMU_SecIMU_CS_Ctl,
                                 SrvIMU_SecIMU_BusTrans_Rec,
                                 SrvOsCommon.delay_ms,
                                 SrvOsCommon.get_os_ms);

            DevICM20602.config(&ICM20602Obj,
                                ICM20602_SampleRate_2K,
                                ICM20602_Acc_16G,
                                ICM20602_Gyr_2000DPS);

            InUse_SecIMU_Obj.obj_ptr = &ICM20602Obj;
            InUse_SecIMU_Obj.OriData_ptr = &(ICM20602Obj.OriData);
            InUse_SecIMU_Obj.acc_trip = ICM20602Obj.PHY_AccTrip_Val;
            InUse_SecIMU_Obj.gyr_trip = ICM20602Obj.PHY_GyrTrip_Val;

            InUse_SecIMU_Obj.set_drdy = DevICM20602.set_ready;
            InUse_SecIMU_Obj.get_drdy = DevICM20602.get_ready;
            InUse_SecIMU_Obj.get_scale = DevICM20602.get_scale;
            InUse_SecIMU_Obj.sample = DevICM20602.sample;
            InUse_SecIMU_Obj.get_error = DevICM20602.get_error;

            if (!DevICM20602.init(&ICM20602Obj))
                return SrvIMU_SecDev_Init_Error;
        break;

        case SrvIMU_Dev_ICM42688P:
            InUse_SecIMU_Obj.type = SrvIMU_Dev_ICM42688P;
            DevICM426xx.pre_init(&ICM42688PObj,
                                 SrvIMU_SecIMU_CS_Ctl,
                                 SrvIMU_SecIMU_BusTrans_Rec,
                                 SrvOsCommon.delay_ms,
                                 SrvOsCommon.get_os_ms);

            DevICM426xx.config(&ICM42688PObj,
                                ICM426xx_SampleRate_2K,
                                ICM426xx_Acc_16G,
                                ICM426xx_Gyr_2000DPS);

            InUse_SecIMU_Obj.obj_ptr = &ICM42688PObj;
            InUse_SecIMU_Obj.OriData_ptr = &(ICM42688PObj.OriData);
            InUse_SecIMU_Obj.acc_trip = ICM42688PObj.PHY_AccTrip_Val;
            InUse_SecIMU_Obj.gyr_trip = ICM42688PObj.PHY_GyrTrip_Val;

            InUse_SecIMU_Obj.set_drdy = DevICM426xx.set_ready;
            InUse_SecIMU_Obj.get_drdy = DevICM426xx.get_ready;
            InUse_SecIMU_Obj.get_scale = DevICM426xx.get_scale;
            InUse_SecIMU_Obj.sample = DevICM426xx.sample;
            InUse_SecIMU_Obj.get_error = DevICM426xx.get_error;

            if (!DevICM426xx.init(&ICM42688PObj))
                return SrvIMU_SecDev_Init_Error;
        break;

        case SrvIMU_Dev_ICM42605:
            InUse_SecIMU_Obj.type = SrvIMU_Dev_ICM42605;
            DevICM426xx.pre_init(&ICM42605Obj,
                                 SrvIMU_SecIMU_CS_Ctl,
                                 SrvIMU_SecIMU_BusTrans_Rec,
                                 SrvOsCommon.delay_ms,
                                 SrvOsCommon.get_os_ms);

            DevICM426xx.config(&ICM42605Obj,
                                ICM426xx_SampleRate_2K,
                                ICM426xx_Acc_16G,
                                ICM426xx_Gyr_2000DPS);

            InUse_SecIMU_Obj.obj_ptr = &ICM42605Obj;
            InUse_SecIMU_Obj.OriData_ptr = &(ICM42605Obj.OriData);
            InUse_SecIMU_Obj.acc_trip = ICM42605Obj.PHY_AccTrip_Val;
            InUse_SecIMU_Obj.gyr_trip = ICM42605Obj.PHY_GyrTrip_Val;

            InUse_SecIMU_Obj.set_drdy = DevICM426xx.set_ready;
            InUse_SecIMU_Obj.get_drdy = DevICM426xx.get_ready;
            InUse_SecIMU_Obj.get_scale = DevICM426xx.get_scale;
            InUse_SecIMU_Obj.sample = DevICM426xx.sample;
            InUse_SecIMU_Obj.get_error = DevICM426xx.get_error;

            if (!DevICM426xx.init(&ICM42605Obj))
                return SrvIMU_SecDev_Init_Error;
        break;

        default: return SrvIMU_SecDev_Detect_Error;
    }

    return SrvIMU_No_Error;
}

/* input true selected / false deselected */
static void SrvIMU_PriIMU_CS_Ctl(bool state)
{
    BspGPIO.write(PriIMU_CSPin.port, PriIMU_CSPin.pin, state);
}

static void SrvIMU_SecIMU_CS_Ctl(bool state)
{
    BspGPIO.write(SecIMU_CSPin.port, SecIMU_CSPin.pin, state);
}

static bool SrvIMU_PriIMU_BusTrans_Rec(uint8_t *Tx, uint8_t *Rx, uint16_t size)
{
    return BspSPI.trans_receive(&PriIMU_Bus_Instance, Tx, Rx, size, IMU_Commu_TimeOut);
}

static bool SrvIMU_SecIMU_BusTrans_Rec(uint8_t *Tx, uint8_t *Rx, uint16_t size)
{
    return BspSPI.trans_receive(&SecIMU_Bus_Instance, Tx, Rx, size, IMU_Commu_TimeOut);
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

static GenCalib_State_TypeList SrvIMU_Set_Calib(uint32_t calib_cycle)
{
    if((Gyro_Calib_Monitor.state != Calib_Start) || (Gyro_Calib_Monitor.state != Calib_InProcess))
    {
        Gyro_Calib_Monitor.state = Calib_Start;
        Gyro_Calib_Monitor.calib_cycle = calib_cycle;
        Gyro_Calib_Monitor.cur_cycle = calib_cycle;
        return Calib_InProcess;
    }

    return Gyro_Calib_Monitor.state;
}

static GenCalib_State_TypeList SrvIMU_Get_Calib(void)
{
    return Gyro_Calib_Monitor.state;
}

static GenCalib_State_TypeList SrvIMU_Calib_GyroZeroOffset(uint32_t calib_cycle, uint32_t *calib_cycle_cnt, float *pri_gyr, float *sec_gyr)
{
    uint8_t i = Axis_X;
    GenCalib_State_TypeList state = Calib_Failed;
    static int16_t lst_pri_gyr[Axis_Sum] = {0};
    static int16_t lst_sec_gyr[Axis_Sum] = {0};
    static int16_t PriIMU_Prc_Gyr_ZeroOffset[Axis_Sum] = {0};
    static int16_t SecIMU_Prc_Gyr_ZeroOffset[Axis_Sum] = {0};

    int16_t pri_gyr_tmp[Axis_Sum] = {0};
    int16_t sec_gyr_tmp[Axis_Sum] = {0};

    if(calib_cycle_cnt == NULL)
        return Calib_Failed;

    if((pri_gyr == NULL) || 
       (sec_gyr == NULL))
        goto reset_calib_var;

    if(*calib_cycle_cnt)
    {
        for(; i < Axis_Sum; i++)
        {
            pri_gyr_tmp[i] = (int16_t)(pri_gyr[i] * GYR_STATIC_CALIB_ACCURACY);
            sec_gyr_tmp[i] = (int16_t)(sec_gyr[i] * GYR_STATIC_CALIB_ACCURACY);

            /* motion detect */
            if((pri_gyr_tmp[i] >= GYR_STATIC_CALIB_ANGULAR_SPEED_THRESHOLD) || 
               (sec_gyr_tmp[i] >= GYR_STATIC_CALIB_ANGULAR_SPEED_THRESHOLD) ||
               (abs(pri_gyr_tmp[i] - lst_pri_gyr[i]) >= GYR_STATIC_CALIB_ANGULAR_SPEED_DIFF_THRESHOLD) ||
               (abs(sec_gyr_tmp[i] - lst_sec_gyr[i]) >= GYR_STATIC_CALIB_ANGULAR_SPEED_DIFF_THRESHOLD))
            {
                /* reset variable */
                state = Calib_Failed;
                goto reset_calib_var;
            }

            /* we need to keep sensor for static statment for entiry calibration proce */

            PriIMU_Prc_Gyr_ZeroOffset[i] += pri_gyr_tmp[i];
            SecIMU_Prc_Gyr_ZeroOffset[i] += sec_gyr_tmp[i];

            lst_pri_gyr[i] = pri_gyr_tmp[i];
            lst_sec_gyr[i] = sec_gyr_tmp[i];
        }

        (*calib_cycle_cnt)--;

        /* calib done */
        if((*calib_cycle_cnt) == 0)
        {
            for(i = Axis_X; i < Axis_Sum; i++)
            {
                PriIMU_Gyr_ZeroOffset[i] = (PriIMU_Prc_Gyr_ZeroOffset[i] / (float)GYR_STATIC_CALIB_ACCURACY) / calib_cycle;
                SecIMU_Gyr_ZeroOffset[i] = (SecIMU_Prc_Gyr_ZeroOffset[i] / (float)GYR_STATIC_CALIB_ACCURACY) / calib_cycle;
            }

            state = Calib_Done;
        }
        else
            return Calib_InProcess;
    }

reset_calib_var:
    /* for test */
    for(i = Axis_X; i < Axis_Sum; i++)
    {
        PriIMU_Prc_Gyr_ZeroOffset[i] = 0;
        SecIMU_Prc_Gyr_ZeroOffset[i] = 0;

        lst_pri_gyr[i] = 0;
        lst_sec_gyr[i] = 0;
    }
            
    return state;
}

static bool SrvIMU_Sample(SrvIMU_SampleMode_List mode)
{
    static uint32_t PriSample_Rt_Lst = 0;
    static uint32_t SecSample_Rt_Lst = 0;
    uint8_t i = Axis_X;
    bool pri_sample_state = true;
    bool sec_sample_state = true;
    IMUModuleScale_TypeDef pri_imu_scale;
    IMUModuleScale_TypeDef sec_imu_scale;
    float Sample_MsDiff = 0.0f;
    bool PriSample_Enable = mode & SrvIMU_Priori_Pri;
    bool SecSample_Enable = mode & SrvIMU_Priori_Sec;

    /* don`t use error tree down below it may decrease code efficient */
    /* trigger error directly when sampling */

    if(mode > SrvIMU_Both_Sample)
        return false;

    /* lock fus data */
    SrvMpu_Update_Reg.sec.Fus_State = true;

    /* pri imu init successed */
    if (SrvMpu_Init_Reg.sec.Pri_State & PriSample_Enable)
    {
        pri_imu_scale = InUse_PriIMU_Obj.get_scale(InUse_PriIMU_Obj.obj_ptr);
        PriIMU_Data.module = SrvIMU_PriModule;
        PriIMU_Data.acc_scale = pri_imu_scale.acc_scale;
        PriIMU_Data.gyr_scale = pri_imu_scale.gyr_scale;

        /* pri imu module data ready triggered */
        if (InUse_PriIMU_Obj.get_drdy(InUse_PriIMU_Obj.obj_ptr) && InUse_PriIMU_Obj.sample(InUse_PriIMU_Obj.obj_ptr))
        {
            /* lock */
            SrvMpu_Update_Reg.sec.Pri_State = true;

            PriIMU_Data.cycle_cnt++;
            PriIMU_Data.time_stamp = InUse_PriIMU_Obj.OriData_ptr->time_stamp;

            /* check Primary IMU module Sample is correct or not */
            if (PriSample_Rt_Lst)
            {
                if (PriIMU_Data.time_stamp <= PriSample_Rt_Lst)
                    pri_sample_state = false;
            }

            if (pri_sample_state)
            {
                /* update pri imu data */
                PriIMU_Data.tempera = InUse_PriIMU_Obj.OriData_ptr->temp_flt;

                /* Pri imu data validation check */
                PriIMU_Data.error_code = SrvIMU_DataCheck(InUse_PriIMU_Obj.OriData_ptr, InUse_PriIMU_Obj.acc_trip, InUse_PriIMU_Obj.gyr_trip);
                Sample_MsDiff = PriIMU_Data.time_stamp - PriSample_Rt_Lst;

                for (i = Axis_X; i < Axis_Sum; i++)
                {
                    PriIMU_Data.org_acc[i] = InUse_PriIMU_Obj.OriData_ptr->acc_flt[i];
                    PriIMU_Data.org_gyr[i] = InUse_PriIMU_Obj.OriData_ptr->gyr_flt[i] - PriIMU_Gyr_ZeroOffset[i];

                    /* filted imu data */
                    PriIMU_Data.flt_gyr[i] = Butterworth.update(PriIMU_Gyr_LPF_Handle[i], PriIMU_Data.org_gyr[i]);
                    PriIMU_Data.flt_acc[i] = Butterworth.update(PriIMU_Acc_LPF_Handle[i], PriIMU_Data.org_acc[i]);

                    /* update last time value */
                    if ((PriIMU_Data.error_code != SrvIMU_Sample_Data_Acc_OverRange) &&
                        (PriIMU_Data.error_code != SrvIMU_Sample_Data_Gyr_OverRange))
                    {
                        InUse_PriIMU_Obj.OriData_ptr->acc_int_lst[i] = InUse_PriIMU_Obj.OriData_ptr->acc_int[i];
                        InUse_PriIMU_Obj.OriData_ptr->gyr_int_lst[i] = InUse_PriIMU_Obj.OriData_ptr->gyr_int[i];
                    
                        /* over angular accelerate error detect */
                        if (SrvIMU_Detect_AngularOverSpeed(PriIMU_Data.org_gyr[i], PriIMU_Data_Lst.org_gyr[i], Sample_MsDiff))
                            PriIMU_Data.error_code = SrvIMU_Sample_Over_Angular_Accelerate;
                    }
                }
                 
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
    if (SrvMpu_Init_Reg.sec.Sec_State & SecSample_Enable)
    {
        sec_imu_scale = InUse_SecIMU_Obj.get_scale(InUse_SecIMU_Obj.obj_ptr);
        SecIMU_Data.module = SrvIMU_SecModule;
        SecIMU_Data.acc_scale = sec_imu_scale.acc_scale;
        SecIMU_Data.gyr_scale = sec_imu_scale.gyr_scale;

        /* sec imu module data ready triggered */
        if (InUse_SecIMU_Obj.get_drdy(InUse_SecIMU_Obj.obj_ptr) && InUse_SecIMU_Obj.sample(InUse_SecIMU_Obj.obj_ptr))
        {
            /* lock */
            SrvMpu_Update_Reg.sec.Sec_State = true;

            SecIMU_Data.cycle_cnt++;
            SecIMU_Data.time_stamp = InUse_SecIMU_Obj.OriData_ptr->time_stamp;

            /* check Secondry IMU module Sample is correct or not */
            if (SecSample_Rt_Lst)
            {
                if (SecIMU_Data.time_stamp <= SecSample_Rt_Lst)
                    sec_sample_state = false;
            }

            if (sec_sample_state)
            {
                /* update sec imu data */
                SecIMU_Data.tempera = InUse_SecIMU_Obj.OriData_ptr->temp_flt;

                /* Sec imu data validation check */
                SecIMU_Data.error_code = SrvIMU_DataCheck(InUse_SecIMU_Obj.OriData_ptr, InUse_SecIMU_Obj.acc_trip, InUse_SecIMU_Obj.gyr_trip);
                Sample_MsDiff = (SecIMU_Data.time_stamp - SecSample_Rt_Lst) / 1000.0f;

                for (i = Axis_X; i < Axis_Sum; i++)
                {
                    SecIMU_Data.org_acc[i] = InUse_SecIMU_Obj.OriData_ptr->acc_flt[i];
                    SecIMU_Data.org_gyr[i] = InUse_SecIMU_Obj.OriData_ptr->gyr_flt[i] - SecIMU_Gyr_ZeroOffset[i];

                    SecIMU_Data.flt_gyr[i] = Butterworth.update(SecIMU_Gyr_LPF_Handle[i], SecIMU_Data.org_gyr[i]);
                    SecIMU_Data.flt_acc[i] = Butterworth.update(SecIMU_Acc_LPF_Handle[i], SecIMU_Data.org_acc[i]);

                    /* update Sec last value */
                    if ((SecIMU_Data.error_code != SrvIMU_Sample_Data_Acc_OverRange) &&
                        (SecIMU_Data.error_code != SrvIMU_Sample_Data_Gyr_OverRange))
                    {
                        InUse_SecIMU_Obj.OriData_ptr->acc_int_lst[i] = InUse_SecIMU_Obj.OriData_ptr->acc_int[i];
                        InUse_SecIMU_Obj.OriData_ptr->gyr_int_lst[i] = InUse_SecIMU_Obj.OriData_ptr->gyr_int[i];
                    
                        if (SrvIMU_Detect_AngularOverSpeed(SecIMU_Data.org_gyr[i], SecIMU_Data_Lst.org_gyr[i], Sample_MsDiff))
                            SecIMU_Data.error_code = SrvIMU_Sample_Over_Angular_Accelerate;
                    }
                }

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

    /* update calibration state */
    if((Gyro_Calib_Monitor.state == Calib_Start) || (Gyro_Calib_Monitor.state == Calib_InProcess))
    {
        Gyro_Calib_Monitor.state = SrvIMU_Calib_GyroZeroOffset(Gyro_Calib_Monitor.calib_cycle, &Gyro_Calib_Monitor.cur_cycle, PriIMU_Data.flt_gyr, SecIMU_Data.flt_gyr);
    }

    switch(mode)
    {
        case SrvIMU_Priori_Pri:
            IMU_Data = PriIMU_Data;
        break;

        case SrvIMU_Priori_Sec:
            IMU_Data = SecIMU_Data;
        break;

        case SrvIMU_Both_Sample:
            if((PriIMU_Data.error_code == SrvIMU_Sample_NoError) || 
               (PriIMU_Data.error_code == SrvIMU_Sample_Data_Acc_Blunt))
            {
                IMU_Data = PriIMU_Data;
            }
            else if((SecIMU_Data.error_code == SrvIMU_Sample_NoError) || 
                    (SecIMU_Data.error_code == SrvIMU_Sample_Data_Acc_Blunt))
            {
                IMU_Data = SecIMU_Data;
            }
            else
            {
                IMU_Data = IMU_Data_Lst;
                IMU_Data.time_stamp = SrvOsCommon.get_os_ms();
            }
        break;

        default:
            return false;
    }

    /* unlock fus data */
    SrvMpu_Update_Reg.sec.Fus_State = false;
    IMU_Data_Lst = IMU_Data;

    return (pri_sample_state | sec_sample_state);
}

static bool SrvIMU_Get_Range(SrvIMU_Module_Type module, SrvIMU_Range_TypeDef *range)
{
    if(((SrvIMU_PriModule == module) || (SrvIMU_SecModule == module)) && range)
    {
        if((SrvIMU_PriModule == module) && SrvMpu_Init_Reg.sec.Pri_State)
        {
            range->Acc = InUse_PriIMU_Obj.acc_trip;
            range->Gyr = InUse_PriIMU_Obj.gyr_trip;
            
            return true; 
        }
        else if((SrvIMU_SecModule == module) && SrvMpu_Init_Reg.sec.Sec_State)
        {
            range->Acc = InUse_SecIMU_Obj.acc_trip;
            range->Gyr = InUse_SecIMU_Obj.gyr_trip;

            return true;
        }
    }

    return false;
}

static bool SrvIMU_Get_Data(SrvIMU_Module_Type type, SrvIMU_Data_TypeDef *data)
{
    SrvIMU_Data_TypeDef imu_data_tmp;

    memset(&imu_data_tmp, NULL, IMU_DATA_SIZE);

    if(data == NULL)
        return false;

reupdate_imu:
    if (type == SrvIMU_PriModule)
    {
        if (!SrvMpu_Update_Reg.sec.Pri_State)
        {
            memcpy(&imu_data_tmp, &PriIMU_Data, IMU_DATA_SIZE);
        }
        else
            goto reupdate_imu_statistics;
    }
    else if (type == SrvIMU_SecModule)
    {
        if (!SrvMpu_Update_Reg.sec.Sec_State)
        {
            memcpy(&imu_data_tmp, &SecIMU_Data, IMU_DATA_SIZE);
        }
        else
            goto reupdate_imu_statistics;
    }
    else if(type == SrvIMU_FusModule)
    {
        if(!SrvMpu_Update_Reg.sec.Fus_State)
        {
            memcpy(&imu_data_tmp, &IMU_Data, IMU_DATA_SIZE);
        }
        else
            goto reupdate_imu_statistics;
    }

    memcpy(data, &imu_data_tmp, sizeof(SrvIMU_Data_TypeDef));
    return true;

reupdate_imu_statistics:
    SrvIMU_Reupdate_Statistics_CNT ++;
    goto reupdate_imu;
}

static float SrvIMU_Get_MaxAngularSpeed_Diff(void)
{
    uint16_t pri_angular_diff = 0;
    uint16_t sec_angular_diff = 0;

    if (SrvMpu_Init_Reg.sec.Pri_State)
    {
        pri_angular_diff = (uint16_t)(DevMPU6000.get_gyr_angular_speed_diff(&MPU6000Obj) * ANGULAR_SPEED_ACCURACY);
    }

    if (SrvMpu_Init_Reg.sec.Sec_State)
    {
        sec_angular_diff = (uint16_t)(DevICM20602.get_gyr_angular_speed_diff(&ICM20602Obj) * ANGULAR_SPEED_ACCURACY);
    }

    if (pri_angular_diff >= sec_angular_diff)
        return pri_angular_diff / (float)ANGULAR_SPEED_ACCURACY;

    return sec_angular_diff / (float)ANGULAR_SPEED_ACCURACY;
}

static bool SrvIMU_Detect_AngularOverSpeed(float angular_speed, float lst_angular_speed, float ms_diff)
{
    uint16_t AngularSpeed_Diff = (uint16_t)(fabs(angular_speed - lst_angular_speed) / ms_diff * ANGULAR_SPEED_ACCURACY);

    if (AngularSpeed_Diff >= (uint16_t)(ANGULAR_ACCECLERATION_THRESHOLD * ANGULAR_SPEED_ACCURACY))
        return true;

    return false;
}

static void SrvIMU_ErrorProc(void)
{
    ErrorLog.proc(SrvMPU_Error_Handle);
}

static SrvIMU_SensorID_List SrvIMU_AutoDetect(bus_trans_callback trans, cs_ctl_callback cs_ctl)
{
    SrvOsCommon.delay_ms(20);

    if(DevMPU6000.detect(trans, cs_ctl))
        return SrvIMU_Dev_MPU6000;

    if(DevICM20602.detect(trans, cs_ctl))
        return SrvIMU_Dev_ICM20602;

    switch(DevICM426xx.detect(trans, cs_ctl))
    {
        case ICM42605: return SrvIMU_Dev_ICM42605;
        case ICM42688P: return SrvIMU_Dev_ICM42688P;
        default: break;
    }

    return SrvIMU_Dev_None;
}

/************************************************************ general function *****************************************************************************/
static char* SrvIMU_GetSensorType_Str(SrvIMU_SensorID_List type)
{
    switch(type)
    {
        case SrvIMU_Dev_MPU6000:
            return "MPU6000\r\n";
        break;

        case SrvIMU_Dev_ICM20602:
            return "ICM20602\r\n";
        break;

        case SrvIMU_Dev_ICM42688P:
            return "ICM42688P\r\n";
        break;

        case SrvIMU_Dev_ICM42605:
            return "ICM42605\r\n";
        break;

        case SrvIMU_Dev_None:
            return "None\r\n";
        break;

        default:
            return "None\r\n";
    }
}

/************************************************************ DataReady Pin Exti Callback *****************************************************************************/
static void SrvIMU_PriIMU_ExtiCallback(void)
{
    if (SrvMpu_Init_Reg.sec.Pri_State)
        InUse_PriIMU_Obj.set_drdy(InUse_PriIMU_Obj.obj_ptr);
}

static void SrvIMU_SecIMU_ExtiCallback(void)
{
    if (SrvMpu_Init_Reg.sec.Sec_State)
        InUse_SecIMU_Obj.set_drdy(InUse_SecIMU_Obj.obj_ptr);
}

/*************************************************************** Error Process Callback *******************************************************************************/
static void SrvIMU_PriDev_Filter_InitError(int16_t code, uint8_t *p_arg, uint16_t size)
{
    ErrorLog.add_desc("IMU Pri Filter Init Error\r\n");
}

static void SrvIMU_SecDev_Filter_InitError(int16_t code, uint8_t *p_arg, uint16_t size)
{
    ErrorLog.add_desc("IMU Sec Filter Init Error\r\n");
}

static void SrvIMU_Dev_InitError(int16_t code, uint8_t *p_arg, uint16_t size)
{
    SrvIMU_InuseSensorObj_TypeDef *InUseObj_Ptr = NULL;

    if(p_arg && size)
    {
        InUseObj_Ptr = (SrvIMU_InuseSensorObj_TypeDef *)p_arg;
        
        ErrorLog.add_desc("IMU   Type:   %s", SrvIMU_GetSensorType_Str(InUseObj_Ptr->type));
        ErrorLog.add_desc("error code:   %d\r\n", InUseObj_Ptr->get_error(InUseObj_Ptr->obj_ptr).code);
        ErrorLog.add_desc("error func:   %s\r\n", InUseObj_Ptr->get_error(InUseObj_Ptr->obj_ptr).function);
        ErrorLog.add_desc("error line:   %d\r\n", InUseObj_Ptr->get_error(InUseObj_Ptr->obj_ptr).line);
        ErrorLog.add_desc("error reg:    %02x\r\n", InUseObj_Ptr->get_error(InUseObj_Ptr->obj_ptr).tar_reg);
        ErrorLog.add_desc("error reg rx: %02x\r\n", InUseObj_Ptr->get_error(InUseObj_Ptr->obj_ptr).reg_r_val);
        ErrorLog.add_desc("error reg tx: %02x\r\n", InUseObj_Ptr->get_error(InUseObj_Ptr->obj_ptr).reg_t_val);

        ErrorLog.add_desc("\r\n");
    }
}

static void SrvIMU_AllModule_InitError(int16_t code, uint8_t *p_arg, uint16_t size)
{
    if(p_arg && size)
        (*(uint32_t *)p_arg)++;
}

static void SrvIMU_PriSample_Undrdy(uint8_t *p_arg, uint16_t size)
{
}

static void SrvIMU_SecSample_Undrdy(uint8_t *p_arg, uint16_t size)
{
}
