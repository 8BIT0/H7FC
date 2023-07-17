#include "Srv_Baro.h"
#include "Srv_OsCommon.h"
#include "IO_Definition.h"
#include "debug_util.h"
#include "bsp_iic.h"
#include "bsp_gpio.h"

#define ToIIC_BusObj(x) (BspIICObj_TypeDef *)x
#define ToIIC_BusAPI(x)

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

/* external function */
static uint8_t SrvBaro_Init(SrvBaroObj_TypeDef *obj, SrvBaro_TypeList type, uint16_t rate);


SrvBaro_TypeDef SrvBaro = {
    .init = SrvBaro_Init,
    .sample = NULL,
    .ready = NULL,
    .get = NULL,
};

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

static uint8_t SrvBaro_Bus_Init(void)
{
    switch((uint8_t)SrvBaroBus.type)
    {
        case SrvBaro_Bus_IIC:

            break;

        default:
            return SrvBaro_Error_BadBusType;
    }
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


