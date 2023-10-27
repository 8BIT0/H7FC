#include "Srv_SensorMonitor.h"
#include "Bsp_Timer.h"
#include "Srv_OsCommon.h"

/* 
 * for sensor statistic function a timer is essenial
 */

/* internal function */
static uint32_t SrvSensorMonitor_Get_FreqVal(uint8_t freq_enum);

static bool SrvSensorMonitor_Init(SrvSensorMonitorObj_TypeDef *obj)
{
    uint32_t freq_tmp = 0;
    uint8_t enable_sensor_num = 0;

    if(obj)
    {
        enable_sensor_num = Get_OnSet_Bit_Num(obj->enabled_reg);
        
        obj->statistic_list = SrvOsCommon.malloc(enable_sensor_num * sizeof(SrvSensorMonitor_Statistic_TypeDef));
        if(obj->statistic_list == NULL)
        {
            SrvOsCommon.free(obj->statistic_list);
            return false;
        }

        /* enabled on imu must be essential */
        if(obj->enabled_reg.bit.imu)
        {
            freq_tmp = SrvSensorMonitor_Get_FreqVal(obj->freq_reg.bit.imu);
        }
        else
            return false;

        if(obj->enabled_reg.bit.mag)
        {
            freq_tmp = SrvSensorMonitor_Get_FreqVal(obj->freq_reg.bit.mag);
        }
        else
            obj->init_state_reg.bit.mag = false;

        if(obj->enabled_reg.bit.baro)
        {
            freq_tmp = SrvSensorMonitor_Get_FreqVal(obj->freq_reg.bit.baro);
        }
        else
            obj->init_state_reg.bit.baro = false;

        if(obj->enabled_reg.bit.tof)
        {
            freq_tmp = SrvSensorMonitor_Get_FreqVal(obj->freq_reg.bit.tof);
        }
        else
            obj->init_state_reg.bit.tof = false;

        if(obj->enbaled_reg.bit.gnss)
        {
            freq_tmp = SrvSensorMonitor_Get_FreqVal(obj->freq_reg.bit.gnss);
        }
        else
            obj->init_state_reg.bit.gnss = false;

        /* enable a 32bit timer for statistic */
    }

    return false;
}

static uint32_t SrvSensorMonitor_Get_FreqVal(uint8_t freq_enum)
{
    switch(freq_enum)
    {
        case SrvSensorMonitor_SampleFreq_1KHz:
            return 1000;

        case SrvSensorMonitor_SampleFreq_500Hz:
            return 500;
        
        case SrvSensorMonitor_SampleFreq_250Hz:
            return 250;

        case SrvSensorMonitor_SampleFreq_200Hz:
            return 200;

        case SrvSensorMonitor_SampleFreq_100Hz:
            return 100;

        case SrvSensorMonitor_SampleFreq_50Hz:
            return 50;

        case SrvSensorMonitor_SampleFreq_20Hz:
            return 20;
        
        case SrvSensorMonitor_SampleFreq_10Hz:
            return 10;

        case SrvSensorMonitor_SampleFreq_5Hz:
            return 5;

        case SrvSensorMonitor_SampleFreq_1Hz:
            return 1;

        default:
            return 0;
    }

    return 0;
}
