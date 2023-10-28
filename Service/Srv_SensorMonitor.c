#include "Srv_SensorMonitor.h"
#include "Bsp_Timer.h"
#include "Srv_OsCommon.h"

/* 
 * for sensor statistic function a timer is essenial
 */
#define SrvSensorMonitor_GetSampleInterval(period) (1000 / period)

/* internal vriable */
SrvSensorMonitor_Statistic_TypeDef *statistic_imu  = NULL;
SrvSensorMonitor_Statistic_TypeDef *statistic_mag  = NULL;
SrvSensorMonitor_Statistic_TypeDef *statistic_baro = NULL;
SrvSensorMonitor_Statistic_TypeDef *statistic_gnss = NULL;
SrvSensorMonitor_Statistic_TypeDef *statistic_tof  = NULL;

/* internal function */
static uint32_t SrvSensorMonitor_Get_FreqVal(uint8_t freq_enum);

static bool SrvSensorMonitor_IMU_Init(void);
static bool SrvSensorMonitor_Mag_Init(void);
static bool SrvSensorMonitor_Baro_Init(void);
static bool SrvSensorMonitor_Gnss_Init(void);
static bool SrvSensorMonitor_Tof_Init(void);

static bool SrvSensorMonitor_Init(SrvSensorMonitorObj_TypeDef *obj)
{
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
        if(obj->enabled_reg.bit.imu && SrvSensorMonitor_IMU_Init())
        {
            obj->init_state_reg.bit.imu = true;
            statistic_imu = &obj->statistic_list[0];
            statistic_imu->set_period = fSrvSensorMonitor_Get_FreqVal(obj->freq_reg.bit.imu);
        }
        else
        {
            if(obj->enabled_reg.bit.imu)
                obj->init_state_reg.bit.imu = false;

            return false;
        }

        if(obj->enabled_reg.bit.mag && SrvSensorMonitor_Mag_Init())
        {
            obj->init_state_reg.bit.mag = true;
            statistic_mag = &obj->statistic_list[1];
            statistic_mag->set_period = SrvSensorMonitor_Get_FreqVal(obj->freq_reg.bit.mag);
        }
        else
        {
            if(obj->enabled_reg.bit.mag)
                obj->init_state_reg.bit.mag = false;

            obj->init_state_reg.bit.mag = false;
        }

        if(obj->enabled_reg.bit.baro && SrvSensorMonitor_Baro_Init())
        {
            obj->init_state_reg.bit.baro = true;
            statistic_baro = obj->statistic_list[2];
            statistic_baro->set_period = SrvSensorMonitor_Get_FreqVal(obj->freq_reg.bit.baro);
        }
        else
        {
            if(obj->enabled_reg.bit.baro)
                obj->init_state_reg.bit.baro = false;
            
            obj->init_state_reg.bit.baro = false;
        }

        if(obj->enabled_reg.bit.tof && SrvSensorMonitor_Tof_Init())
        {
            obj->init_state_reg.bit.tof = true;
            statistic_tof = obj->statistic_list[3];
            statistic_tof->set_period = SrvSensorMonitor_Get_FreqVal(obj->freq_reg.bit.tof);
        }
        else
        {
            if(obj->enabled_reg.bit.tof)
                obj->init_state_reg.bit.tof = false;

            obj->init_state_reg.bit.tof = false;
        }

        if(obj->enbaled_reg.bit.gnss && SrvSensorMonitor_Gnss_Init())
        {
            obj->init_state_reg.bit.gnss = true;
            statistic_gnss = obj->statistic_list[4];
            statistic_gnss->set_period = SrvSensorMonitor_Get_FreqVal(obj->freq_reg.bit.gnss);
        }
        else
        {
            if(obj->enabled_reg.bit.gnss)
                obj->init_state_reg.bit.tof = false;

            obj->init_state_reg.bit.gnss = false;
        }

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

static uint32_t SrvSensorMonitor_Get_EnableState(SrvSensorMonitorObj_TypeDef *obj)
{
    if(obj)
        return obj->enabled_reg.val;

    return 0;
}

static uint32_t SrvSensorMonitor_Get_InitState(SrvSensorMonitorObj_TypeDef *obj)
{
    if(obj)
        return obj->init_state_reg.val;

    return 0;
}

/******************************************* IMU Section **********************************************/
static bool SrvSensorMonitor_IMU_Init(void)
{
    if(SrvIMU.init && (SrvIMU.init() != SrvIMU_AllModule_Init_Error))
        return true;
    
    return false;
}

static bool SrvSensorMonitor_IMU_SampleCTL(SrvSensorMonitorObj_TypeDef *obj)
{
    uint32_t sample_interval_ms = 0;
    uint32_t cur_time = SrvOsCommon.get_os_ms();

    if(obj && statistic_imu && obj->enabled_reg.bit.imu && obj->init_state_reg.bit.imu && statistic_imu->set_period)
    {
        sample_interval_ms = SrvSensorMonitor_GetSampleInterval(statistic_imu->set_period);
        
        if( SrvIMU.sample && \
            SrvIMU.error_proc && 
            ((statistic_imu->start_time == 0) || \
             (sample_interval_ms && \
             (statistic_imu->nxt_sample_time >= cur_time))))
        {
            SrvIMU.sample(SrvIMU_FusModule);
            SrvIMU.error_proc();

            statistic_imu->nxt_sample_time = cur_time + sample_interval_ms;
            if(statistic_imu->start_time == 0)
                statistic_imu->start_time = cur_time;

            return true;
        }
    }

    return false;
}

static SrvIMU_UnionData_TypeDef SrvSensorMonitor_Get_IMUData(SrvSensorMonitorObj_TypeDef *obj)
{
    SrvIMU_UnionData_TypeDef imu_data_tmp;

    memset(&imu_data_tmp, 0, sizeof(SrvIMU_UnionData_TypeDef));

    if(obj && obj->enabled_reg.bit.imu && obj->init_state_reg.bit.imu && SrvIMU.get_data)
    {
        imu_data_tmp = SrvIMU.get_data(SrvIMU_FusModule);
    }

    return imu_data_tmp;
}

/******************************************* Mag Section **********************************************/
/* still in developing */
static bool SrvSensorMonitor_Mag_Init(void)
{
    if(SrvBaro.init && (SrvBaro.init() == SrvBaro_Error_None))
        return true;

    return false;
}

static bool SrvSensorMonitot_Mag_SampleCTL(SrvSensorMonitorObj_TypeDef *obj)
{
    if(obj && obj->enabled_reg.bit.mag && obj->init_state_reg.bit.mag)
    {
        
    }

    return false;
}

static uint32_t SrvSensorMonitor_Get_MagData(SrvSensorMonitorObj_TypeDef *obj)
{
    return 0;
}
/******************************************* Baro Section *********************************************/
/* still in developing */
static bool SrvSensorMonitor_Baro_Init(void)
{
    return false;
}

static bool SrvSensorMonitor_Baro_SampleCTL(SrvSensorMonitorObj_TypeDef *obj)
{
    if(obj && obj->enabled_reg.bit.baro && obj->init_state_reg.bit.baro)
    {
        
    }

    return false;
}

static SrvBaroData_TypeDef SrvSensorMonitor_Get_BaroData(SrvSensorMonitorObj_TypeDef *obj)
{
    SrvBaroData_TypeDef baro_data_tmp;

    memset(&baro_data_tmp, 0, sizeof(SrvBaroData_TypeDef));

    if(obj && obj->enabled_reg.bit.baro && obj->init_state_reg.bit.baro)
    {
    }

    return baro_data_tmp;
}

/******************************************* Gnss Section *********************************************/
/* still in developing */
static bool SrvSensorMonitor_Gnss_Init(void)
{
    return false;
}

static bool SrvSensorMonitor_Gnss_SampleCTL(SrvSensorMonitorObj_TypeDef *obj)
{
    if(obj && obj->enabled_reg.bit.gnss && obj->init_state_reg.bit.gnss)
    {
        
    }

    return false;
}

/******************************************* Tof Section **********************************************/
/* still in developing */
static bool SrvSensorMonitor_Tof_Init(void)
{
    return false;
}

staitc bool SrvSensorMonitor_Tof_SampleCTL(SrvSensorMonitorObj_TypeDef *obj)
{
    if(obj && obj->enabled_reg.bit.gnss && obj->init_state_reg.bit.gnss)
    {
        
    }

    return false;
}

static bool SrvSensorMonitor_SampleCTL(void)
{

    return false;
}
