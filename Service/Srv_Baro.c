/*
 * Auther: 8_B!T0
 * Baro Sensor Sample Service
 * this file still can be optimized in a big way
 * for currently only one baro sensor can be used
 */
#include "Srv_Baro.h"
#include "Srv_OsCommon.h"
#include "../FCHW_Config.h"
#include "error_log.h"
#include "bsp_iic.h"
#include "bsp_gpio.h"
#include "HW_Def.h"
#include <math.h>

#define STANDER_ATMOSPHERIC_PRESSURE (101.325f * 1000)
#define SRVBARO_SMOOTHWINDOW_SIZE 5

#define SRVBARO_MAX_SAMPLE_PERIOD 10    // unit: ms 10ms 100hz
#define SRVBARO_MIN_SAMPLE_PERIOD 100   // unit: ms 100ms 10hz

/* internal vriable */
SrvBaroObj_TypeDef SrvBaroObj = {
    .type = Baro_Type_DPS310,   /* default set */
    .sample_rate = SRVBARO_SAMPLE_RATE_20HZ,
    .init_err = SrvBaro_Error_None,
    .sensor_data = NULL,
};

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
static bool SrvBaro_Get_Date(SrvBaro_UnionData_TypeDef *data);
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
    SrvBaroBus.init = false;
    SrvBaroBus.bus_obj = (void *)&Baro_BusCfg;
    SrvBaroBus.type = bus_type;
    
    switch((uint8_t)bus_type)
    {
        case SrvBaro_Bus_IIC:
            SrvBaroBus.bus_api = (void *)&BspIIC;

#if defined STM32H743xx
            ToIIC_BusObj(SrvBaroBus.bus_obj)->handle = SrvOsCommon.malloc(I2C_HandleType_Size);
            if (ToIIC_BusObj(SrvBaroBus.bus_obj)->handle == NULL)
            {
                SrvOsCommon.free(ToIIC_BusObj(SrvBaroBus.bus_obj)->handle);
                return false;
            }

            ToIIC_BusObj(SrvBaroBus.bus_obj)->PeriphClkInitStruct = SrvOsCommon.malloc(I2C_PeriphCLKInitType_Size);
            if (ToIIC_BusObj(SrvBaroBus.bus_obj)->PeriphClkInitStruct == NULL)
            {
                SrvOsCommon.free(ToIIC_BusObj(SrvBaroBus.bus_obj)->handle);
                SrvOsCommon.free(ToIIC_BusObj(SrvBaroBus.bus_obj)->PeriphClkInitStruct);
                return false;
            }
#elif defined AT32F435_437
            ToIIC_BusObj(SrvBaroBus.bus_obj)->handle = SrvOsCommon.malloc(sizeof(I2C_Handle));
            if (ToIIC_BusObj(SrvBaroBus.bus_obj)->handle == NULL)
            {
                SrvOsCommon.free(ToIIC_BusObj(SrvBaroBus.bus_obj)->handle);
                return false;
            }
#endif
            if (!ToIIC_BusAPI(SrvBaroBus.bus_api)->init(ToIIC_BusObj(SrvBaroBus.bus_obj)))
                return false;

            SrvBaroBus.init = true;
            return true;

        case SrvBaro_Bus_SPI:
            SrvBaroBus.bus_api = (void *)&BspSPI;

            /* spi cs pin init */
            if (!BspGPIO.out_init(*p_Baro_CS))
                return false;

            /* spi bus port init */
            if (!ToSPI_BusAPI(SrvBaroBus.bus_api)->init(To_NormalSPI_Obj(SrvBaroBus.bus_obj), &Baro_Bus_Instance))
                return false;

            /* set cs pin high */
            BspGPIO.write(*p_Baro_CS, true);
            SrvBaroBus.init = true;
            
            return true;

        default: return false;
    }

    return false;
}

static uint8_t SrvBaro_Init(SrvBaro_TypeList sensor_type, SrvBaroBus_TypeList bus_type)
{
    /* create error log handle */
    SrvBaro_Error_Handle = ErrorLog.create("SrvBaro_Error");

    /* regist all error to the error tree */
    ErrorLog.registe(SrvBaro_Error_Handle, SrvBaro_ErrorList, sizeof(SrvBaro_ErrorList) / sizeof(SrvBaro_ErrorList[0]));

    if (!SrvBaro_BusInit(bus_type))
    {
        ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_BusInit, NULL, 0);
        return SrvBaro_Error_BusInit;
    }

    if (SrvBaroObj.sample_rate)
    {
        switch((uint8_t)sensor_type)
        {
            case Baro_Type_DPS310:
                SrvBaroObj.type = sensor_type;
                SrvBaroObj.sensor_obj = SrvOsCommon.malloc(sizeof(DevDPS310Obj_TypeDef));
                SrvBaroObj.sensor_api = &DevDPS310;

                if ((SrvBaroObj.sensor_obj != NULL) &&
                    (SrvBaroObj.sensor_api != NULL))
                {
                    /* set sensor object */
                    switch (SrvBaroBus.type)
                    {
                        case SrvBaro_Bus_IIC:
                            ToDPS310_OBJ(SrvBaroObj.sensor_obj)->DevAddr = DPS310_I2C_ADDR;
                            ToDPS310_OBJ(SrvBaroObj.sensor_obj)->bus_rx = (DevDPS310_BusRead)SrvBaro_IICBus_Rx;
                            ToDPS310_OBJ(SrvBaroObj.sensor_obj)->bus_tx = (DevDPS310_BusWrite)SrvBaro_IICBus_Tx;
                            break;

                        case SrvBaro_Bus_SPI:
                        default: return SrvBaro_Error_BusType;
                    }

                    ToDPS310_OBJ(SrvBaroObj.sensor_obj)->get_tick = SrvOsCommon.get_os_ms;
                    ToDPS310_OBJ(SrvBaroObj.sensor_obj)->bus_delay = SrvOsCommon.delay_ms;

                    /* device init */
                    if (!ToDPS310_API(SrvBaroObj.sensor_api)->init(ToDPS310_OBJ(SrvBaroObj.sensor_obj)))
                    {
                        ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_DevInit, NULL, 0);
                        return SrvBaro_Error_DevInit;
                    }

                    SrvBaroObj.calib_state = Calib_Start;
                    SrvBaroObj.calib_cycle = SRVBARO_DEFAULT_CALI_CYCLE;

                    SrvBaroObj.data_size = DPS310_DataSize;
                    SrvBaroObj.sensor_data = SrvOsCommon.malloc(SrvBaroObj.data_size);
                }
                else
                {
                    ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_BadSensorObj, NULL, 0);
                    return SrvBaro_Error_BadSensorObj;
                }
                break;

            case Baro_Type_BMP280:
                SrvBaroObj.type = sensor_type;
                SrvBaroObj.sensor_obj = SrvOsCommon.malloc(sizeof(DevBMP280Obj_TypeDef));
                SrvBaroObj.sensor_api = &DevBMP280;

                if ((SrvBaroObj.sensor_obj != NULL) && \
                    (SrvBaroObj.sensor_api != NULL))
                {
                    ToBMP280_OBJ(SrvBaroObj.sensor_obj)->send = NULL;
                    ToBMP280_OBJ(SrvBaroObj.sensor_obj)->recv = NULL;
                    ToBMP280_OBJ(SrvBaroObj.sensor_obj)->trans = NULL;
                    ToBMP280_OBJ(SrvBaroObj.sensor_obj)->bus_tx = NULL;
                    ToBMP280_OBJ(SrvBaroObj.sensor_obj)->bus_rx = NULL;

                    /* set sensor object */
                    switch (SrvBaroBus.type)
                    {
                        case SrvBaro_Bus_SPI:
                            ToBMP280_OBJ(SrvBaroObj.sensor_obj)->send = (DevBMP280_BusCommu)SrvBaro_SPIBus_Tx;
                            ToBMP280_OBJ(SrvBaroObj.sensor_obj)->recv = (DevBMP280_BusCommu)SrvBaro_SPIBus_Rx;
                            ToBMP280_OBJ(SrvBaroObj.sensor_obj)->trans = (DevBMP280_Trans)SrvBaro_SPIBus_Trans;
                            break;
                        
                        case SrvBaro_Bus_IIC:
                            ToBMP280_OBJ(SrvBaroObj.sensor_obj)->DevAddr = BMP280_ADDR_1;
                            ToBMP280_OBJ(SrvBaroObj.sensor_obj)->bus_tx = (DevBMP280_IIC_Write)SrvBaro_IICBus_Tx;
                            ToBMP280_OBJ(SrvBaroObj.sensor_obj)->bus_rx = (DevBMP280_IIC_Read)SrvBaro_IICBus_Rx;
                            break;

                        default: return SrvBaro_Error_BusType;
                    }

                    ToBMP280_OBJ(SrvBaroObj.sensor_obj)->get_tick = SrvOsCommon.get_os_ms;
                    ToBMP280_OBJ(SrvBaroObj.sensor_obj)->delay_ms = SrvOsCommon.delay_ms;

                    /* device init */
                    if (!ToBMP280_API(SrvBaroObj.sensor_api)->init(ToBMP280_OBJ(SrvBaroObj.sensor_obj)))
                    {
                        ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_DevInit, NULL, 0);
                        return SrvBaro_Error_DevInit;
                    }

                    SrvBaroObj.calib_state = Calib_Start;
                    SrvBaroObj.calib_cycle = SRVBARO_DEFAULT_CALI_CYCLE;
                    
                    SrvBaroObj.data_size = BMP280_DataSize;
                    SrvBaroObj.sensor_data = SrvOsCommon.malloc(SrvBaroObj.data_size);
                }
                else
                {
                    ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_BadSensorObj, NULL, 0);
                    return SrvBaro_Error_BadSensorObj;
                }
                break;
            
            default:
                ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_BadType, NULL, 0);
                return SrvBaro_Error_BadType;
        }

        SrvBaroObj.sample_period = round(1000.0 / (double)SrvBaroObj.sample_rate);

        /* filter init */
        SrvBaroObj.smoothwindow_filter_hdl = SmoothWindow.init(SRVBARO_SMOOTHWINDOW_SIZE);
        if (SrvBaroObj.smoothwindow_filter_hdl == 0)
        {
            ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_FilterInit, NULL, 0);
            return SrvBaro_Error_FilterInit;
        }

        if ((SrvBaroObj.sample_period < SRVBARO_MAX_SAMPLE_PERIOD) || \
            (SrvBaroObj.sample_period > SRVBARO_MIN_SAMPLE_PERIOD))
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

    return SrvBaro_Error_None;
}

static bool SrvBaro_Sample(void)
{
    if (SrvBaroObj.init_err == SrvBaro_Error_None)
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

                        if (SrvBaroObj.calib_cycle == 0)
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

                    if (((SrvBaroObj.calib_state == Calib_Start) || \
                        (SrvBaroObj.calib_state == Calib_InProcess)) && \
                        SrvBaroObj.calib_cycle)
                    {
                        SrvBaroObj.pressure_add_sum += ToBMP280_OBJ(SrvBaroObj.sensor_obj)->pressure;
                        SrvBaroObj.pressure_add_sum /= 2;

                        SrvBaroObj.calib_cycle --;

                        if (SrvBaroObj.calib_cycle == 0)
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
    if (SrvBaroObj.calib_state != Calib_InProcess)
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

static bool SrvBaro_Get_Date(SrvBaro_UnionData_TypeDef *data)
{
    SrvBaro_UnionData_TypeDef baro_data_tmp;
    float alt = 0.0f;
    uint8_t check_sum = 0;

    memset(baro_data_tmp.buff, 0, sizeof(SrvBaro_UnionData_TypeDef));

    if ((SrvBaroObj.init_err == SrvBaro_Error_None) && data && SrvBaroObj.sensor_data && SrvBaroObj.data_size)
    {
        switch ((uint8_t) SrvBaroObj.type)
        {
            case Baro_Type_DPS310:
                if (ToDPS310_API(SrvBaroObj.sensor_api)->ready(ToDPS310_OBJ(SrvBaroObj.sensor_obj)))
                {
                    memset(SrvBaroObj.sensor_data, 0, DPS310_DataSize);
                    *ToDPS310_DataPtr(SrvBaroObj.sensor_data) = ToDPS310_API(SrvBaroObj.sensor_api)->get_data(ToDPS310_OBJ(SrvBaroObj.sensor_obj));
                    
                    /* convert baro pressure to meter */
                    alt = SrvBaro_PessureCnvToMeter(ToDPS310_DataPtr(SrvBaroObj.sensor_data)->scaled_press);

                    baro_data_tmp.data.time_stamp = ToDPS310_DataPtr(SrvBaroObj.sensor_data)->time_stamp;
                    baro_data_tmp.data.cyc = ToDPS310_DataPtr(SrvBaroObj.sensor_data)->cyc;
                    baro_data_tmp.data.pressure_alt_offset = SrvBaroObj.alt_offset;
                    baro_data_tmp.data.pressure_alt = alt - SrvBaroObj.alt_offset;
                    baro_data_tmp.data.tempra = ToDPS310_DataPtr(SrvBaroObj.sensor_data)->scaled_tempra;
                    baro_data_tmp.data.pressure = ToDPS310_DataPtr(SrvBaroObj.sensor_data)->scaled_press;

                    /* doing baro filter */
                    baro_data_tmp.data.pressure_alt = SmoothWindow.update(SrvBaroObj.smoothwindow_filter_hdl, baro_data_tmp.data.pressure_alt);
                    for(uint8_t i = 0; i < sizeof(baro_data_tmp.buff) - sizeof(baro_data_tmp.data.check_sum); i++)
                        check_sum += baro_data_tmp.buff[i];
                    baro_data_tmp.data.check_sum = check_sum;
                    memcpy(data, &baro_data_tmp, sizeof(SrvBaroData_TypeDef));
                    
                    return true;
                }
                break;
            
            case Baro_Type_BMP280:
                if (ToBMP280_API(SrvBaroObj.sensor_api)->ready(ToBMP280_OBJ(SrvBaroObj.sensor_obj)))
                {
                    memset(SrvBaroObj.sensor_data, 0, BMP280_DataSize);
                    *ToBMP280_DataPtr(SrvBaroObj.sensor_data) = ToBMP280_API(SrvBaroObj.sensor_api)->get_data(ToBMP280_OBJ(SrvBaroObj.sensor_obj));
                    
                    /* convert baro pressure to meter */
                    alt = SrvBaro_PessureCnvToMeter(ToBMP280_DataPtr(SrvBaroObj.sensor_data)->scaled_press);

                    baro_data_tmp.data.time_stamp = ToBMP280_DataPtr(SrvBaroObj.sensor_data)->time_stamp;
                    baro_data_tmp.data.cyc = ToBMP280_DataPtr(SrvBaroObj.sensor_data)->cyc;
                    baro_data_tmp.data.pressure_alt_offset = SrvBaroObj.alt_offset;
                    baro_data_tmp.data.pressure_alt = alt - SrvBaroObj.alt_offset;
                    baro_data_tmp.data.tempra = ToBMP280_DataPtr(SrvBaroObj.sensor_data)->scaled_tempra;
                    baro_data_tmp.data.pressure = ToBMP280_DataPtr(SrvBaroObj.sensor_data)->scaled_press;

                    /* doing baro filter */
                    baro_data_tmp.data.pressure_alt = SmoothWindow.update(SrvBaroObj.smoothwindow_filter_hdl, baro_data_tmp.data.pressure_alt);
                    for (uint8_t i = 0; i < sizeof(baro_data_tmp.buff) - sizeof(baro_data_tmp.data.check_sum); i++)
                        check_sum += baro_data_tmp.buff[i];
                    baro_data_tmp.data.check_sum = check_sum;
                    memcpy(data, &baro_data_tmp, sizeof(SrvBaroData_TypeDef));
                    
                    return true;
                }
                break;

            default: break;
        }
    }

    return false;
}

/*************************************************************** Bus Comunicate Callback *******************************************************************************/
static bool SrvBaro_IICBus_Tx(uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint8_t len)
{
    BspIICObj_TypeDef *IICBusObj = NULL;

    if (SrvBaroBus.init && ((p_data != NULL) || (len != 0)))
    {
        IICBusObj = ToIIC_BusObj(SrvBaroBus.bus_obj);
        return ToIIC_BusAPI(SrvBaroBus.bus_api)->write(IICBusObj, dev_addr << 1, reg_addr, p_data, len);
    }

    return false;
}

static bool SrvBaro_IICBus_Rx(uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint8_t len)
{
    BspIICObj_TypeDef *IICBusObj = NULL;

    if (SrvBaroBus.init && ((p_data != NULL) || (len != 0)))
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
        (p_Baro_CS != NULL) && \
        (SrvBaroBus.type == SrvBaro_Bus_SPI))
    {
        /* CS Low */
        BspGPIO.write(*p_Baro_CS, false);

        if (ToSPI_BusAPI(SrvBaroBus.bus_api)->trans_receive(&Baro_Bus_Instance, p_tx, p_rx, len, 100))
            res = len;

        /* CS High */
        BspGPIO.write(*p_Baro_CS, true);
    }

    return res;
}

static uint16_t SrvBaro_SPIBus_Tx(uint8_t *p_data, uint16_t len)
{
    uint16_t res = 0;

    if (p_data && len && \
        SrvBaroBus.init && \
        (p_Baro_CS != NULL) && \
        (SrvBaroBus.type == SrvBaro_Bus_SPI))
    {
        /* CS Low */
        BspGPIO.write(*p_Baro_CS, false);

        if (ToSPI_BusAPI(SrvBaroBus.bus_api)->trans(&Baro_Bus_Instance, p_data, len, 100))
            res = len;

        /* CS High */
        BspGPIO.write(*p_Baro_CS, true);
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
        BspGPIO.write(*p_Baro_CS, false);

        if (ToSPI_BusAPI(SrvBaroBus.bus_api)->receive(&Baro_Bus_Instance, p_data, len, 100))
            res = len;
        
        /* CS High */
        BspGPIO.write(*p_Baro_CS, true);
    }
    
    return res;
}

/*************************************************************** Error Process Callback *******************************************************************************/
static void SrvBaro_BusInitError(int16_t code, uint8_t *p_arg, uint16_t size)
{
    switch(code)
    {
        case SrvBaro_Error_BadRate: break;
        case SrvBaro_Error_BadType: break;
        case SrvBaro_Error_BadSensorObj: break;
        case SrvBaro_Error_BadSamplePeriod: break;
        case SrvBaro_Error_BusInit: break;
        case SrvBaro_Error_DevInit: break;
        case SrvBaro_Error_FilterInit: break;
        default:
            ErrorLog.add_desc("SrvBaro Triggered Unknow ErrorCode: %d\r\n", code);
        break;
    }
}
