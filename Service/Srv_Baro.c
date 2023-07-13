#include "Srv_Baro.h"
#include "Srv_OsCommon.h"
#include "IO_Definition.h"
#include "debug_util.h"
#include "bsp_iic.h"

#define SRVBARO_MAX_SAMPLE_PERIOD 10    // unit: ms 10ms 100hz
#define SRVBARO_MIN_SAMPLE_PERIOD 100   // unit: ms 100ms 10hz

/* external function */
static uint8_t SrvBaro_Init(SrvBaroObj_TypeDef *obj, SrvBaro_TypeList type, uint16_t rate);


SrvBaro_TypeDef SrvBaro = {
    .init = SrvBaro_Init,
    .sample = NULL,
    .ready = NULL,
    .get = NULL,
};

static bool SrvBaro_AttachBus(SrvBaroBus_TypeList bus_type, void *bus_obj)
{
    if((bus_type < SrvBaro_BusType_Sum) && bus_obj)
    {
        switch(bus_type)
        {
            case SrvBaro_Bus_IIC:
            break;

            case SrvBaro_Bus_SPI:
            break;

            default:
                return false;
        }
    }

    return false;
}

static uint8_t SrvBaro_Init(SrvBaroObj_TypeDef *obj, SrvBaro_TypeList type, uint16_t rate)
{
    if(obj)
    {
        memset(obj, 0, sizeof(SrvBaroObj_TypeDef));

        if(obj->bus_handle == 0)
            return SrvBaro_Error_BusHandle;

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
                    return SrvBaro_Error_BadSensorObj;

                break;
                
                default:
                    return SrvBaro_Error_BadType;
            }

            obj->type = type;
            obj->sample_rate = rate;
            obj->sample_period = round(1000.0 / (double)rate);

            if((obj->sample_period > SRVBARO_MAX_SAMPLE_PERIOD) || (obj->sample_period < SRVBARO_MIN_SAMPLE_PERIOD))
                return SrvBaro_Error_BadSamplePeriod;
        }
        else
            return SrvBaro_Error_BadRate;
    }
    else
        return SrvBaro_Error_BadObj;

    return SrvBaro_Error_None;
}

static bool SrvBaro_I2C_Tx(uint8_t dev_addr, uint8_t *p_data, uint8_t len, bool ack)
{
    if((p_data != NULL) || (len != 0))
    {
    }

    return false;
}

static bool SrvBaro_I2C_Rx(uint8_t dev_addr, uint8_t *p_data, uint8_t len, bool ack)
{
    if((p_data != NULL) || (len != 0))
    {

    }

    return false;
}


