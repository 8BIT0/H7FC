#include "Srv_Baro.h"
#include "Srv_OsCommon.h"
#include "IO_Definition.h"
#include "debug_util.h"
#include "error_log.h"
#include "bsp_iic.h"
#include "bsp_gpio.h"

#define ToIIC_BusObj(x) ((BspIICObj_TypeDef *)x)
#define ToIIC_BusAPI(x) ((BspIIC_TypeDef *)x)

#define SRVBARO_MAX_SAMPLE_PERIOD 10    // unit: ms 10ms 100hz
#define SRVBARO_MIN_SAMPLE_PERIOD 100   // unit: ms 100ms 10hz

BspIICObj_TypeDef SrvBaro_IIC_Obj = {
    .init = false,
    .instance_id = BspIIC_Instance_I2C_2,
    .Pin = &SrvBaro_BusPin,
};

SrvBaroBusObj_TypeDef SrvBaroBus = {
    .type = SrvBaro_Bus_IIC,
    .bus_obj = (void *)&SrvBaro_IIC_Obj,
    .bus_api = (void *)&BspIIC,
};

/************************************************************************ Error Tree Item ************************************************************************/
static Error_Handler SrvBaro_Error_Handle = NULL;

static void SrvBaro_BusInitError(int16_t code, uint8_t *p_arg, uint16_t size);

static Error_Obj_Typedef SrvBaro_ErrorList[] = {
    {
        .out = true,
        .log = false,
        .prc_callback = SrvBaro_BusInitError,
        .code = SrvBaro_Error_BadObj,
        .desc = "SrvBaro Bad Service Object\r\n",
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
    }
};
/************************************************************************ Error Tree Item ************************************************************************/

/* external function */
static uint8_t SrvBaro_Init(SrvBaroObj_TypeDef *obj, SrvBaro_TypeList type, uint16_t rate);


SrvBaro_TypeDef SrvBaro = {
    .init = SrvBaro_Init,
    .sample = NULL,
    .ready = NULL,
    .get = NULL,
};

static bool SrvBaro_BusInit(void)
{
    if(SrvBaroBus.bus_obj && SrvBaroBus.bus_api)
    {
        switch((uint8_t)SrvBaroBus.type)
        {
            case SrvBaro_Bus_IIC:
                if(ToIIC_BusAPI(SrvBaroBus.bus_api)->init(ToIIC_BusObj(SrvBaroBus.bus_obj)))
                    return true;
                break;

            default:
                return false;
        }
    }

    return false;
}

static uint8_t SrvBaro_Init(SrvBaroObj_TypeDef *obj, SrvBaro_TypeList type, uint16_t rate)
{
    /* create error log handle */
    SrvBaro_Error_Handle = ErrorLog.create("SrvBaro_Error");

    /* regist all error to the error tree */
    ErrorLog.registe(SrvBaro_Error_Handle, SrvBaro_ErrorList, sizeof(SrvBaro_ErrorList) / sizeof(SrvBaro_ErrorList[0]));

    if(obj)
    {
        memset(obj, 0, sizeof(SrvBaroObj_TypeDef));

        if(!SrvBaro_BusInit())
        {
            ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_BusInit, NULL, 0);
            return SrvBaro_Error_BusInit;
        }

        if(rate)
        {
            switch((uint8_t)type)
            {
                case Baro_Type_DPS310:
                // obj->sensor_obj = SrvOsCommon.malloc();
                // obj->sensor_api = ;
                if((obj->sensor_obj != NULL) &&
                   (obj->sensor_api != NULL))
                {
                    /* device init */
                    // if()
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

            obj->type = type;
            obj->sample_rate = rate;
            obj->sample_period = round(1000.0 / (double)rate);

            if((obj->sample_period > SRVBARO_MAX_SAMPLE_PERIOD) || (obj->sample_period < SRVBARO_MIN_SAMPLE_PERIOD))
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
    }
    else
    {
        ErrorLog.trigger(SrvBaro_Error_Handle, SrvBaro_Error_BadObj, NULL, 0);
        return SrvBaro_Error_BadObj;
    }

    return SrvBaro_Error_None;
}

static bool SrvBaro_Bus_Tx(uint8_t dev_addr, uint8_t *p_data, uint8_t len, bool ack)
{
    if((p_data != NULL) || (len != 0))
    {
    }

    return false;
}

static bool SrvBaro_Bus_Rx(uint8_t dev_addr, uint8_t *p_data, uint8_t len, bool ack)
{
    if((p_data != NULL) || (len != 0))
    {
    }

    return false;
}

/*************************************************************** Error Process Callback *******************************************************************************/
static void SrvBaro_BusInitError(int16_t code, uint8_t *p_arg, uint16_t size)
{
    switch(code)
    {
        case SrvBaro_Error_BadObj:
        case SrvBaro_Error_BadRate:
        case SrvBaro_Error_BadType:
        case SrvBaro_Error_BadSensorObj:
        case SrvBaro_Error_BadSamplePeriod:
        case SrvBaro_Error_BusInit:
        break;

        default:
            ErrorLog.add_desc("SrvBaro Triggered Unknow ErrorCode: %d\r\n", code);
        break;
    }
}