/*
 * Auther: 8_B!T0
 * Baro Sensor Sample Service
 * this file still can be optimized in a big way
 * for currently only one baro sensor can be used
 */
#include "Srv_Baro.h"
#include "Srv_OsCommon.h"
#include "../FCHW_Config.h"
#include "HW_Def.h"
#include "debug_util.h"
#include "error_log.h"
#include "bsp_iic.h"
#include "bsp_gpio.h"
#include <math.h>

#define STANDER_ATMOSPHERIC_PRESSURE (101.325f * 1000)
#define SRVBARO_SMOOTHWINDOW_SIZE 5

#define SRVBARO_MAX_SAMPLE_PERIOD 10    // unit: ms 10ms 100hz
#define SRVBARO_MIN_SAMPLE_PERIOD 100   // unit: ms 100ms 10hz

#define BARO_TAG "[ BARO INFO ] "
#define BARO_INFO(fmt, ...) Debug_Print(&DebugP4, BARO_TAG, fmt, ##__VA_ARGS__)

/* internal vriable */
SrvBaroObj_TypeDef SrvBaroObj = {
    .type = Baro_Type_DPS310,
    .sample_rate = SRVBARO_SAMPLE_RATE_20HZ,
    .init_err = SrvBaro_Error_None,
};

#if defined MATEKH743_V1_5

#if defined  STM32H743xx
static SPI_HandleTypeDef Baro_Bus_Instance;
#endif

BspIICObj_TypeDef SrvBaro_IIC_Obj = {
    .init = false,
    .instance_id = BARO_BUS,
    .Pin = &SrvBaro_BusPin,
};

#elif defined AT32F435RGT7
static void *Baro_Bus_Instance = NULL;
#endif

SrvBaroBusObj_TypeDef SrvBaroBus = {
    .type = SrvBaro_Bus_None,
    .init = false,
    .bus_obj = NULL,
    .bus_api = NULL,
};

/* internal function */
static bool SrvBaro_IICBus_Tx(uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint8_t len);
static bool SrvBaro_IICBus_Rx(uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint8_t len);

static uint16_t SrvBaro_SPIBus_Tx(uint8_t *p_data, uint16_t len);
static uint16_t SrvBaro_SPIBus_Rx(uint8_t *p_data, uint16_t len);
static uint16_t SrvBaro_SPIBus_Trans(uint8_t *p_tx, uint8_t *p_rx, uint16_t len);

static float SrvBaro_PessureCnvToMeter(float pa);

/************************************************************************ Error Tree Item ************************************************************************/
static Error_Handler SrvBaro_Error_Handle = 0;

static void SrvBaro_BusInitError(int16_t code, uint8_t *p_arg, uint16_t size);

static Error_Obj_Typedef SrvBaro_ErrorList[] = {
    {
        .out = true,
        .log = false,
        .prc_callback = SrvBaro_BusInitError,
        .code = SrvBaro_Error_BadRate,
        .desc = "SrvBaro Bad Sample Rate\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = SrvBaro_BusInitError,
        .code = SrvBaro_Error_BadType,
        .desc = "SrvBaro Bad Sensor Type\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = SrvBaro_BusInitError,
        .code = SrvBaro_Error_BadSensorObj,
        .desc = "SrvBaro Bad Sensor Object\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = SrvBaro_BusInitError,
        .code = SrvBaro_Error_BadSamplePeriod,
        .desc = "SrvBaro Bad Sample Period\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = SrvBaro_BusInitError,
        .code = SrvBaro_Error_BusInit,
        .desc = "SrvBaro Bus Init Failed\r\n",
        .proc_type = Error_Proc_Ignore,
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = SrvBaro_BusInitError,
        .code = SrvBaro_Error_FilterInit,
        .desc = "SrvBaro Sensor Filter Init Failed\r\n",
        .proc_type = Error_Proc_Immd, 
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
    {
        .out = true,
        .log = false,
        .prc_callback = SrvBaro_BusInitError,
        .code = SrvBaro_Error_DevInit,
        .desc = "SrvBaro Sensor Device Init Failed\r\n",
        .proc_type = Error_Proc_Immd, 
        .prc_data_stream = {
            .p_data = NULL,
            .size = 0,
        },
    },
};
/************************************************************************ Error Tree Item ************************************************************************/

/* external function */
static uint8_t SrvBaro_Init(SrvBaro_TypeList sensor_type, SrvBaroBus_TypeList bus_type);
static bool SrvBaro_Sample(void);
static bool SrvBaro_Get_Date(SrvBaroData_TypeDef *data);
static GenCalib_State_TypeList SrvBaro_Set_Calib(uint16_t cyc);
static GenCalib_State_TypeList SrvBaro_Get_Calib(void);

SrvBaro_TypeDef SrvBaro = {
    .init = SrvBaro_Init,
    .sample = SrvBaro_Sample,
    .get_data = SrvBaro_Get_Date,
    .set_calib = SrvBaro_Set_Calib,
    .get_calib = SrvBaro_Get_Calib,
};

static bool SrvBaro_BusInit(SrvBaroBus_TypeList bus_type)
{
    BARO_INFO("Init\r\n");

    switch((uint8_t)bus_type)
    {
        case SrvBaro_Bus_IIC:
            BARO_INFO("Bus type IIC\r\n");
            SrvBaroBus.type = bus_type;
            SrvBaroBus.bus_api = (void *)&BspIIC;
#if defined STM32H743xx
            SrvBaroBus.bus_obj = (void *)&SrvBaro_IIC_Obj;

            ToIIC_BusObj(SrvBaroBus.bus_obj)->handle = SrvOsCommon.malloc(I2C_HandleType_Size);
            if(ToIIC_BusObj(SrvBaroBus.bus_obj)->handle == NULL)
            {
                SrvOsCommon.free(ToIIC_BusObj(SrvBaroBus.bus_obj)->handle);
                return false;
            }

            ToIIC_BusObj(SrvBaroBus.bus_obj)->PeriphClkInitStruct = SrvOsCommon.malloc(I2C_PeriphCLKInitType_Size);
            if(ToIIC_BusObj(SrvBaroBus.bus_obj)->PeriphClkInitStruct == NULL)
            {
                SrvOsCommon.free(ToIIC_BusObj(SrvBaroBus.bus_obj)->handle);
                SrvOsCommon.free(ToIIC_BusObj(SrvBaroBus.bus_obj)->PeriphClkInitStruct);
                return false;
            }
#endif
            if(!ToIIC_BusAPI(SrvBaroBus.bus_api)->init(ToIIC_BusObj(SrvBaroBus.bus_obj)))
                return false;

            SrvBaroBus.init = true;
            return true;

        case SrvBaro_Bus_SPI:
            BARO_INFO("Bus type SPI\r\n");
            SrvBaroBus.type = bus_type;

            Baro_BusCfg.Pin = Baro_BusPin;
            SrvBaroBus.bus_api = (void *)&BspSPI;
            SrvBaroBus.bus_obj = (void *)&Baro_BusCfg;

            /* spi cd pin init */
            if (!BspGPIO.out_init(Baro_CSPin))
            {
                BARO_INFO("CS Pin Init Failed\r\n");
                return false;
            }

            /* spi bus port init */
            if (!ToSPI_BusAPI(SrvBaroBus.bus_api)->init(To_NormalSPI_Obj(SrvBaroBus.bus_obj), &Baro_Bus_Instance))
            {
                BARO_INFO("Bus Init Failed\r\n");
                return false;
            }

            /* set cs pin high */
            BspGPIO.write(Baro_CSPin, true);
            SrvBaroBus.init = true;
            BARO_INFO("Bus Init Successed\r\n");
            return true;

        default:
            return false;
    }

    return false;
}

static uint8_t SrvBaro_Init(SrvBaro_TypeList sensor_type, SrvBaroBus_TypeList bus_type)
{
    /* create error log handle */
    SrvBaro_Error_Handle = ErrorLog.create("SrvBaro_Error");

    /* regist all error to the error tree */
    ErrorLog.registe(SrvBaro_Error_Handle, SrvBaro_ErrorList, sizeof(SrvBaro_ErrorList) / sizeof(SrvBaro_ErrorList[0]));

    if(!SrvBaro_BusInit(bus_type))
    {
        ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_BusInit, NULL, 0);
        return SrvBaro_Error_BusInit;
    }

    if(SrvBaroObj.sample_rate)
    {
        switch((uint8_t)sensor_type)
        {
            case Baro_Type_DPS310:
                BARO_INFO("Module type DPS310\r\n");            
                SrvBaroObj.type = sensor_type;
                SrvBaroObj.sensor_obj = SrvOsCommon.malloc(sizeof(DevDPS310Obj_TypeDef));
                SrvBaroObj.sensor_api = &DevDPS310;

                if((SrvBaroObj.sensor_obj != NULL) &&
                    (SrvBaroObj.sensor_api != NULL))
                {
                    /* set sensor object */
                    ToDPS310_OBJ(SrvBaroObj.sensor_obj)->DevAddr = DPS310_I2C_ADDR;
                    ToDPS310_OBJ(SrvBaroObj.sensor_obj)->bus_rx = (DevDPS310_BusRead)SrvBaro_IICBus_Rx;
                    ToDPS310_OBJ(SrvBaroObj.sensor_obj)->bus_tx = (DevDPS310_BusWrite)SrvBaro_IICBus_Tx;
                    ToDPS310_OBJ(SrvBaroObj.sensor_obj)->get_tick = SrvOsCommon.get_os_ms;
                    ToDPS310_OBJ(SrvBaroObj.sensor_obj)->bus_delay = SrvOsCommon.delay_ms;

                    /* device init */
                    BARO_INFO("Module Init\r\n");
                    if(!ToDPS310_API(SrvBaroObj.sensor_api)->init(ToDPS310_OBJ(SrvBaroObj.sensor_obj)))
                    {
                        BARO_INFO("Module Init Failed\r\n");
                        ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_DevInit, NULL, 0);
                        return SrvBaro_Error_DevInit;
                    }

                    BARO_INFO("Module init accomplished\r\n");
                    SrvBaroObj.calib_state = Calib_Start;
                    SrvBaroObj.calib_cycle = SRVBARO_DEFAULT_CALI_CYCLE;
                }
                else
                {
                    BARO_INFO("Object Malloc Faield\r\n");
                    ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_BadSensorObj, NULL, 0);
                    return SrvBaro_Error_BadSensorObj;
                }
                break;

            case Baro_Type_BMP280:
                BARO_INFO("Module Type BMP280\r\n");
                SrvBaroObj.type = sensor_type;
                SrvBaroObj.sensor_obj = SrvOsCommon.malloc(sizeof(DevBMP280Obj_TypeDef));
                SrvBaroObj.sensor_api = &DevBMP280;

                if ((SrvBaroObj.sensor_obj != NULL) && \
                    (SrvBaroObj.sensor_api != NULL))
                {
                    /* set sensor object */
                    ToBMP280_OBJ(SrvBaroObj.sensor_obj)->send = SrvBaro_SPIBus_Tx;
                    ToBMP280_OBJ(SrvBaroObj.sensor_obj)->recv = SrvBaro_SPIBus_Rx;
                    ToBMP280_OBJ(SrvBaroObj.sensor_obj)->trans = SrvBaro_SPIBus_Trans;
                    ToBMP280_OBJ(SrvBaroObj.sensor_obj)->get_tick = SrvOsCommon.get_os_ms;
                    ToBMP280_OBJ(SrvBaroObj.sensor_obj)->delay_ms = SrvOsCommon.delay_ms;

                    BARO_INFO("Module init\r\n");
                    /* device init */
                    if(!ToBMP280_API(SrvBaroObj.sensor_api)->init(ToBMP280_OBJ(SrvBaroObj.sensor_obj)))
                    {
                        BARO_INFO("Module Init Failed\r\n");
                        ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_DevInit, NULL, 0);
                        return SrvBaro_Error_DevInit;
                    }

                    BARO_INFO("Module init accomplished\r\n");
                    SrvBaroObj.calib_state = Calib_Start;
                    SrvBaroObj.calib_cycle = SRVBARO_DEFAULT_CALI_CYCLE;
                }
                else
                {
                    BARO_INFO("Object Malloc Faield\r\n");
                    ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_BadSensorObj, NULL, 0);
                    return SrvBaro_Error_BadSensorObj;
                }
                break;
            
            default:
                ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_BadType, NULL, 0);
                return SrvBaro_Error_BadType;
        }

        BARO_INFO("Sample rate: %d\r\n", SrvBaroObj.sample_rate);
        SrvBaroObj.sample_period = round(1000.0 / (double)SrvBaroObj.sample_rate);
        BARO_INFO("Sample period: %d\r\n", SrvBaroObj.sample_period);

        /* filter init */
        BARO_INFO("Smooth window filter init\r\n");
        SrvBaroObj.smoothwindow_filter_hdl = SmoothWindow.init(SRVBARO_SMOOTHWINDOW_SIZE);
        if(SrvBaroObj.smoothwindow_filter_hdl == 0)
        {
            BARO_INFO("Smooth window filter init failed\r\n");
            ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_FilterInit, NULL, 0);
            return SrvBaro_Error_FilterInit;
        }

        if((SrvBaroObj.sample_period < SRVBARO_MAX_SAMPLE_PERIOD) || (SrvBaroObj.sample_period > SRVBARO_MIN_SAMPLE_PERIOD))
        {
            ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_BadSamplePeriod, NULL, 0);
            return SrvBaro_Error_BadSamplePeriod;
        }
    }
    else
    {
        ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_BadRate, NULL, 0);
        return SrvBaro_Error_BadRate;
    }

    BARO_INFO("Init Successed\r\n");
    return SrvBaro_Error_None;
}

static bool SrvBaro_Sample(void)
{
    if(SrvBaroObj.init_err == SrvBaro_Error_None)
    {
        switch((uint8_t) SrvBaroObj.type)
        {
            case Baro_Type_DPS310:
                if (ToDPS310_API(SrvBaroObj.sensor_api)->sample(ToDPS310_OBJ(SrvBaroObj.sensor_obj)))
                {
                    SrvBaroObj.sample_cnt ++;

                    if (((SrvBaroObj.calib_state == Calib_Start) || 
                        (SrvBaroObj.calib_state == Calib_InProcess)) &&
                        SrvBaroObj.calib_cycle)
                    {
                        SrvBaroObj.pressure_add_sum += ToDPS310_OBJ(SrvBaroObj.sensor_obj)->pressure;
                        SrvBaroObj.pressure_add_sum /= 2;

                        SrvBaroObj.calib_cycle --;

                        if(SrvBaroObj.calib_cycle == 0)
                        {
                            SrvBaroObj.calib_state = Calib_Done;
                            SrvBaroObj.alt_offset = SrvBaro_PessureCnvToMeter(SrvBaroObj.pressure_add_sum);
                            SrvBaroObj.pressure_add_sum = 0.0f;
                        }
                        else
                            SrvBaroObj.calib_state = Calib_InProcess;
                    }

                    return true;
                }
                else
                {
                    SrvBaroObj.sample_err_cnt ++;
                    return false;
                }

            case  Baro_Type_BMP280:
                if (ToBMP280_API(SrvBaroObj.sensor_api)->sample(ToBMP280_OBJ(SrvBaroObj.sensor_obj)))
                {
                    SrvBaroObj.sample_cnt ++;

                    if (((SrvBaroObj.calib_state == Calib_Start) || 
                        (SrvBaroObj.calib_state == Calib_InProcess)) &&
                        SrvBaroObj.calib_cycle)
                    {
                        SrvBaroObj.pressure_add_sum += ToBMP280_OBJ(SrvBaroObj.sensor_obj)->pressure;
                        SrvBaroObj.pressure_add_sum /= 2;

                        SrvBaroObj.calib_cycle --;

                        if(SrvBaroObj.calib_cycle == 0)
                        {
                            SrvBaroObj.calib_state = Calib_Done;
                            SrvBaroObj.alt_offset = SrvBaro_PessureCnvToMeter(SrvBaroObj.pressure_add_sum);
                            SrvBaroObj.pressure_add_sum = 0.0f;
                        }
                        else
                            SrvBaroObj.calib_state = Calib_InProcess;
                    }

                    return true;
                }
                else
                {
                    SrvBaroObj.sample_err_cnt ++;
                    return false;
                }

            default:
                return false;
        }
    }

    return false;
}

static GenCalib_State_TypeList SrvBaro_Set_Calib(uint16_t cyc)
{
    if(SrvBaroObj.calib_state != Calib_InProcess)
    {
        SrvBaroObj.calib_cycle = cyc;
        SrvBaroObj.calib_state = Calib_Start;
        SrvBaroObj.alt_offset = 0.0f;
        return Calib_InProcess;
    }

    return SrvBaroObj.calib_state;
}

static GenCalib_State_TypeList SrvBaro_Get_Calib(void)
{
    return SrvBaroObj.calib_state;
}

static float SrvBaro_PessureCnvToMeter(float pa)
{
    return ((1 - pow((pa / STANDER_ATMOSPHERIC_PRESSURE), 0.1903))) * 44330;
}

static bool SrvBaro_Get_Date(SrvBaroData_TypeDef *data)
{
    SrvBaroData_TypeDef baro_data_tmp;
    DevDPS310_Data_TypeDef DPS310_Data;
    DevBMP280_Data_TypeDef BMP280_Data;
    float alt = 0.0f;

    memset(&baro_data_tmp, 0, sizeof(baro_data_tmp));

    if((SrvBaroObj.init_err == SrvBaro_Error_None) && data)
    {
        switch((uint8_t) SrvBaroObj.type)
        {
            case Baro_Type_DPS310:
                if(ToDPS310_API(SrvBaroObj.sensor_api)->ready(ToDPS310_OBJ(SrvBaroObj.sensor_obj)))
                {
                    memset(&DPS310_Data, 0, sizeof(DevDPS310_Data_TypeDef));
                    DPS310_Data = ToDPS310_API(SrvBaroObj.sensor_api)->get_data(ToDPS310_OBJ(SrvBaroObj.sensor_obj));
                    
                    /* convert baro pressure to meter */
                    alt = SrvBaro_PessureCnvToMeter(DPS310_Data.scaled_press);

                    baro_data_tmp.time_stamp = DPS310_Data.time_stamp;
                    baro_data_tmp.pressure_alt_offset = SrvBaroObj.alt_offset;
                    baro_data_tmp.pressure_alt = alt - SrvBaroObj.alt_offset;
                    baro_data_tmp.tempra = DPS310_Data.scaled_tempra;
                    baro_data_tmp.pressure = DPS310_Data.scaled_press;

                    /* doing baro filter */
                    baro_data_tmp.pressure_alt = SmoothWindow.update(SrvBaroObj.smoothwindow_filter_hdl, baro_data_tmp.pressure_alt);
                    memcpy(data, &baro_data_tmp, sizeof(SrvBaroData_TypeDef));
                    
                    return true;
                }
                break;
            
            case Baro_Type_BMP280:
                if(ToBMP280_API(SrvBaroObj.sensor_api)->ready(ToBMP280_OBJ(SrvBaroObj.sensor_obj)))
                {
                    memset(&BMP280_Data, 0, sizeof(DevBMP280_Data_TypeDef));
                    BMP280_Data = ToBMP280_API(SrvBaroObj.sensor_api)->get_data(ToBMP280_OBJ(SrvBaroObj.sensor_obj));
                    
                    /* convert baro pressure to meter */
                    alt = SrvBaro_PessureCnvToMeter(BMP280_Data.scaled_press);

                    baro_data_tmp.time_stamp = BMP280_Data.time_stamp;
                    baro_data_tmp.pressure_alt_offset = SrvBaroObj.alt_offset;
                    baro_data_tmp.pressure_alt = alt - SrvBaroObj.alt_offset;
                    baro_data_tmp.tempra = BMP280_Data.scaled_tempra;
                    baro_data_tmp.pressure = BMP280_Data.scaled_press;

                    /* doing baro filter */
                    baro_data_tmp.pressure_alt = SmoothWindow.update(SrvBaroObj.smoothwindow_filter_hdl, baro_data_tmp.pressure_alt);
                    memcpy(data, &baro_data_tmp, sizeof(SrvBaroData_TypeDef));
                    
                    return true;
                }
                break;

            default:
                break;
        }
    }

    return false;
}

/*************************************************************** Bus Comunicate Callback *******************************************************************************/
static bool SrvBaro_IICBus_Tx(uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint8_t len)
{
    BspIICObj_TypeDef *IICBusObj = NULL;

    if(SrvBaroBus.init && ((p_data != NULL) || (len != 0)))
    {
        IICBusObj = ToIIC_BusObj(SrvBaroBus.bus_obj);
        return ToIIC_BusAPI(SrvBaroBus.bus_api)->write(IICBusObj, dev_addr << 1, reg_addr, p_data, len);
    }

    return false;
}

static bool SrvBaro_IICBus_Rx(uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint8_t len)
{
    BspIICObj_TypeDef *IICBusObj = NULL;

    if(SrvBaroBus.init && ((p_data != NULL) || (len != 0)))
    {
        IICBusObj = ToIIC_BusObj(SrvBaroBus.bus_obj);
        return ToIIC_BusAPI(SrvBaroBus.bus_api)->read(IICBusObj, dev_addr << 1, reg_addr, p_data, len);
    }

    return false;
}

static uint16_t SrvBaro_SPIBus_Trans(uint8_t *p_tx, uint8_t *p_rx, uint16_t len)
{
    uint16_t res = 0;

    if (p_tx && p_rx && len && \
        SrvBaroBus.init && \
        (SrvBaroBus.type == SrvBaro_Bus_SPI))
    {
        /* CS Low */
        BspGPIO.write(Baro_CSPin, false);

        if (ToSPI_BusAPI(SrvBaroBus.bus_api)->trans_receive(&Baro_Bus_Instance, p_tx, p_rx, len, 100))
            res = len;

        /* CS High */
        BspGPIO.write(Baro_CSPin, true);
    }

    return res;
}

static uint16_t SrvBaro_SPIBus_Tx(uint8_t *p_data, uint16_t len)
{
    uint16_t res = 0;

    if (p_data && len && \
        SrvBaroBus.init && \
        (SrvBaroBus.type == SrvBaro_Bus_SPI))
    {
        /* CS Low */
        BspGPIO.write(Baro_CSPin, false);

        if (ToSPI_BusAPI(SrvBaroBus.bus_api)->trans(&Baro_Bus_Instance, p_data, len, 100))
            res = len;

        /* CS High */
        BspGPIO.write(Baro_CSPin, true);
    }

    return res;
}

static uint16_t SrvBaro_SPIBus_Rx(uint8_t *p_data, uint16_t len)
{
    uint16_t res = 0;

    if (p_data && len && \
        SrvBaroBus.init && \
        (SrvBaroBus.type == SrvBaro_Bus_SPI))
    {
        /* CS Low */
        BspGPIO.write(Baro_CSPin, false);

        if (ToSPI_BusAPI(SrvBaroBus.bus_api)->receive(&Baro_Bus_Instance, p_data, len, 100))
            res = len;
        
        /* CS High */
        BspGPIO.write(Baro_CSPin, true);
    }

    return res;
}

/*************************************************************** Error Process Callback *******************************************************************************/
static void SrvBaro_BusInitError(int16_t code, uint8_t *p_arg, uint16_t size)
{
    switch(code)
    {
        case SrvBaro_Error_BadRate:
        break;

        case SrvBaro_Error_BadType:
        break;

        case SrvBaro_Error_BadSensorObj:
        break;

        case SrvBaro_Error_BadSamplePeriod:
        break;

        case SrvBaro_Error_BusInit:
        break;

        case SrvBaro_Error_DevInit:
        break;

        case SrvBaro_Error_FilterInit:
        break;

        default:
            ErrorLog.add_desc("SrvBaro Triggered Unknow ErrorCode: %d\r\n", code);
        break;
    }
}
