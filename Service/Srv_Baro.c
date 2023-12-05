#include "Srv_Baro.h"
#include "Srv_OsCommon.h"
#include "IO_Definition.h"
#include "debug_util.h"
#include "error_log.h"
#include "bsp_iic.h"
#include "bsp_gpio.h"
#include <math.h>

#define STANDER_ATMOSPHERIC_PRESSURE (101.325f * 1000)
#define SRVBARO_SMOOTHWINDOW_SIZE 5

#define ToIIC_BusObj(x) ((BspIICObj_TypeDef *)x)
#define ToIIC_BusAPI(x) ((BspIIC_TypeDef *)x)

#define ToDPS310_Obj(x) ((DevDPS310Obj_TypeDef *)x)
#define ToDPS310_API(x) ((DevDPS310_TypeDef *)x)

#define SRVBARO_MAX_SAMPLE_PERIOD 10    // unit: ms 10ms 100hz
#define SRVBARO_MIN_SAMPLE_PERIOD 100   // unit: ms 100ms 10hz

/* internal vriable */
SrvBaroObj_TypeDef SrvBaroObj = {
    .type = Baro_Type_DPS310,
    .sample_rate = SRVBARO_SAMPLE_RATE_20HZ,
    .init_err = SrvBaro_Error_None,
};

BspIICObj_TypeDef SrvBaro_IIC_Obj = {
    .init = false,
    .instance_id = BspIIC_Instance_I2C_2,
    .Pin = &SrvBaro_BusPin,
};

SrvBaroBusObj_TypeDef SrvBaroBus = {
    .type = SrvBaro_Bus_IIC,
    .init = false,
    .bus_obj = (void *)&SrvBaro_IIC_Obj,
    .bus_api = (void *)&BspIIC,
};

/* internal function */
static bool SrvBaro_Bus_Tx(uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint8_t len);
static bool SrvBaro_Bus_Rx(uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint8_t len);

static float SrvBaro_PessureCnvToMeter(float pa);

/************************************************************************ Error Tree Item ************************************************************************/
static Error_Handler SrvBaro_Error_Handle = NULL;

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
static uint8_t SrvBaro_Init(void);
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

static bool SrvBaro_BusInit(void)
{
    if(SrvBaroBus.bus_obj && SrvBaroBus.bus_api)
    {
        switch((uint8_t)SrvBaroBus.type)
        {
            case SrvBaro_Bus_IIC:
                if(ToIIC_BusAPI(SrvBaroBus.bus_api)->init(ToIIC_BusObj(SrvBaroBus.bus_obj)))
                {
                    SrvBaroBus.init = true;
                    return true;
                }
                break;

            default:
                return false;
        }
    }

    return false;
}

static uint8_t SrvBaro_Init(void)
{
    /* create error log handle */
    SrvBaro_Error_Handle = ErrorLog.create("SrvBaro_Error");

    /* regist all error to the error tree */
    ErrorLog.registe(SrvBaro_Error_Handle, SrvBaro_ErrorList, sizeof(SrvBaro_ErrorList) / sizeof(SrvBaro_ErrorList[0]));

    if(!SrvBaro_BusInit())
    {
        ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_BusInit, NULL, 0);
        return SrvBaro_Error_BusInit;
    }

    if(SrvBaroObj.sample_rate)
    {
        switch((uint8_t)SrvBaroObj.type)
        {
            case Baro_Type_DPS310:
            SrvBaroObj.sensor_obj = SrvOsCommon.malloc(sizeof(DevDPS310Obj_TypeDef));
            SrvBaroObj.sensor_api = &DevDPS310;

            if((SrvBaroObj.sensor_obj != NULL) &&
                (SrvBaroObj.sensor_api != NULL))
            {
                /* set sensor object */
                memset(SrvBaroObj.sensor_obj, 0, sizeof(DevDPS310Obj_TypeDef));

                ToDPS310_Obj(SrvBaroObj.sensor_obj)->DevAddr = DPS310_I2C_ADDR;
                ToDPS310_Obj(SrvBaroObj.sensor_obj)->bus_rx = SrvBaro_Bus_Rx;
                ToDPS310_Obj(SrvBaroObj.sensor_obj)->bus_tx = SrvBaro_Bus_Tx;
                ToDPS310_Obj(SrvBaroObj.sensor_obj)->get_tick = SrvOsCommon.get_os_ms;
                ToDPS310_Obj(SrvBaroObj.sensor_obj)->bus_delay = SrvOsCommon.delay_ms;

                /* device init */
                if(!ToDPS310_API(SrvBaroObj.sensor_api)->init(ToDPS310_Obj(SrvBaroObj.sensor_obj)))
                {
                    ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_DevInit, NULL, 0);
                    return SrvBaro_Error_DevInit;
                }

                SrvBaroObj.calib_state = Calib_Start;
                SrvBaroObj.calib_cycle = SRVBARO_DEFAULT_CALI_CYCLE;
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
        if(SrvBaroObj.smoothwindow_filter_hdl == 0)
        {
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

    return SrvBaro_Error_None;
}

static bool SrvBaro_Sample(void)
{
    if(SrvBaroObj.init_err == SrvBaro_Error_None)
    {
        switch((uint8_t) SrvBaroObj.type)
        {
            case Baro_Type_DPS310:
                if(ToDPS310_API(SrvBaroObj.sensor_api)->sample(ToDPS310_Obj(SrvBaroObj.sensor_obj)))
                {
                    SrvBaroObj.sample_cnt ++;

                    if(((SrvBaroObj.calib_state == Calib_Start) || 
                        (SrvBaroObj.calib_state == Calib_InProcess)) &&
                        SrvBaroObj.calib_cycle)
                    {
                        SrvBaroObj.pressure_add_sum += ToDPS310_Obj(SrvBaroObj.sensor_obj)->pressure;
                        SrvBaroObj.pressure_add_sum /= 2;

                        SrvBaroObj.calib_cycle --;

                        if(SrvBaroObj.calib_cycle == 0)
                        {
                            SrvBaroObj.calib_state = Calib_Done;
                            SrvBaroObj.alt_offset = SrvBaro_PessureCnvToMeter(SrvBaroObj.pressure_add_sum);
                            SrvBaroObj.pressure_add_sum = 0.0f;
                        }
                    }
                    return true;
                }
                else
                    SrvBaroObj.sample_err_cnt ++;

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
    float alt = 0.0f;

    memset(&baro_data_tmp, 0, sizeof(baro_data_tmp));

    if((SrvBaroObj.init_err == SrvBaro_Error_None) && data)
    {
        switch((uint8_t) SrvBaroObj.type)
        {
            case Baro_Type_DPS310:
                if(ToDPS310_API(SrvBaroObj.sensor_api)->ready(ToDPS310_Obj(SrvBaroObj.sensor_obj)))
                {
                    memset(&DPS310_Data, 0, sizeof(DevDPS310_Data_TypeDef));
                    DPS310_Data = ToDPS310_API(SrvBaroObj.sensor_api)->get_data(ToDPS310_Obj(SrvBaroObj.sensor_obj));
                    
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

            default:
                break;
        }
    }

    return false;
}

/*************************************************************** Bus Comunicate Callback *******************************************************************************/
static bool SrvBaro_Bus_Tx(uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint8_t len)
{
    BspIICObj_TypeDef *IICBusObj = NULL;

    if(SrvBaroBus.init && ((p_data != NULL) || (len != 0)))
    {
        IICBusObj = ToIIC_BusObj(SrvBaroBus.bus_obj);
        return ToIIC_BusAPI(SrvBaroBus.bus_api)->write(IICBusObj, dev_addr << 1, reg_addr, p_data, len);
    }

    return false;
}

static bool SrvBaro_Bus_Rx(uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint8_t len)
{
    BspIICObj_TypeDef *IICBusObj = NULL;

    if(SrvBaroBus.init && ((p_data != NULL) || (len != 0)))
    {
        IICBusObj = ToIIC_BusObj(SrvBaroBus.bus_obj);
        return ToIIC_BusAPI(SrvBaroBus.bus_api)->read(IICBusObj, dev_addr << 1, reg_addr, p_data, len);
    }

    return false;
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
