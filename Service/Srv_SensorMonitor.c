#include "Srv_SensorMonitor.h"
#include "Bsp_Timer.h"
#include "Srv_OsCommon.h"
#include "debug_util.h"
#include "IO_Definition.h"

/* 
 * for sensor statistic function a timer is essenial
 */
#define SrvSensorMonitor_GetSampleInterval(period) (1000 / period)

/* internal function */
static uint32_t SrvSensorMonitor_Get_FreqVal(uint8_t freq_enum);

static bool SrvSensorMonitor_IMU_Init(void);
static bool SrvSensorMonitor_Mag_Init(void);
static bool SrvSensorMonitor_Baro_Init(void);
static bool SrvSensorMonitor_Gnss_Init(void);
static bool SrvSensorMonitor_Tof_Init(void);

/* external function */
static bool SrvSensorMonitor_Init(SrvSensorMonitorObj_TypeDef *obj);
static bool SrvSensorMonitor_SampleCTL(SrvSensorMonitorObj_TypeDef *obj);
static SrvIMU_UnionData_TypeDef SrvSensorMonitor_Get_IMUData(SrvSensorMonitorObj_TypeDef *obj);
static SrvSensorMonitor_CaliState_List SrvSensorMonitor_Set_Module_Calib(SrvSensorMonitorObj_TypeDef *obj, SrvSensorMonitor_Type_List type);
static SrvSensorMonitor_CaliState_List SrvSensorMonitor_Get_Module_Calib(SrvSensorMonitorObj_TypeDef *obj, SrvSensorMonitor_Type_List type);
static SrvBaroData_TypeDef SrvSensorMonitor_Get_BaroData(SrvSensorMonitorObj_TypeDef *obj);

SrvSensorMonitor_TypeDef SrvSensorMonitor = {
    .init = SrvSensorMonitor_Init,
    .sample_ctl = SrvSensorMonitor_SampleCTL,
    .get_imu_data = SrvSensorMonitor_Get_IMUData,
    .get_baro_data = SrvSensorMonitor_Get_BaroData,
    .set_calib = SrvSensorMonitor_Set_Module_Calib,
    .get_calib = SrvSensorMonitor_Get_Module_Calib,
};

static bool SrvSensorMonitor_Init(SrvSensorMonitorObj_TypeDef *obj)
{
    uint8_t enable_sensor_num = 0;

    if(obj)
    {
        enable_sensor_num = Get_OnSet_Bit_Num(obj->enabled_reg.val);
        
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
            obj->statistic_imu = &obj->statistic_list[0];
            obj->statistic_imu->set_period = SrvSensorMonitor_Get_FreqVal(obj->freq_reg.bit.imu);
            obj->statistic_imu->is_calid = Sensor_Calib_None; 
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
            obj->statistic_mag = &obj->statistic_list[1];
            obj->statistic_mag->set_period = SrvSensorMonitor_Get_FreqVal(obj->freq_reg.bit.mag);
            obj->statistic_baro->is_calid = Sensor_Calib_None;
        }
        else
        {
            obj->statistic_mag->is_calid = Sensor_Calib_None;
            obj->init_state_reg.bit.mag = false;
        }

        if(obj->enabled_reg.bit.baro && SrvSensorMonitor_Baro_Init())
        {
            obj->init_state_reg.bit.baro = true;
            obj->statistic_baro = &obj->statistic_list[2];
            obj->statistic_baro->set_period = SrvSensorMonitor_Get_FreqVal(obj->freq_reg.bit.baro);
        }
        else
        {
            obj->statistic_baro->is_calid = Sensor_Calib_None;
            obj->init_state_reg.bit.baro = false;
        }

        if(obj->enabled_reg.bit.tof && SrvSensorMonitor_Tof_Init())
        {
            obj->init_state_reg.bit.tof = true;
            obj->statistic_tof = &obj->statistic_list[3];
            obj->statistic_tof->set_period = SrvSensorMonitor_Get_FreqVal(obj->freq_reg.bit.tof);
        }
        else
        {
            obj->init_state_reg.bit.tof = false;
        }

        if(obj->enabled_reg.bit.gnss && SrvSensorMonitor_Gnss_Init())
        {
            obj->init_state_reg.bit.gnss = true;
            obj->statistic_gnss = &obj->statistic_list[4];
            obj->statistic_gnss->set_period = SrvSensorMonitor_Get_FreqVal(obj->freq_reg.bit.gnss);
        }
        else
        {
            obj->init_state_reg.bit.gnss = false;
        }
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
    bool state = false;
    uint32_t sample_interval_ms = 0;
    uint32_t cur_time = SrvOsCommon.get_os_ms();
    uint32_t start_tick = 0;
    uint32_t end_tick = 0;
    int32_t sample_tick = 0;

    if(obj && obj->statistic_imu && obj->enabled_reg.bit.imu && obj->init_state_reg.bit.imu && obj->statistic_imu->set_period)
    {
        sample_interval_ms = SrvSensorMonitor_GetSampleInterval(obj->statistic_imu->set_period);
        
        if( SrvIMU.sample && \
            SrvIMU.error_proc && 
            ((obj->statistic_imu->start_time == 0) || \
             (sample_interval_ms && \
             (cur_time >= obj->statistic_imu->nxt_sample_time))))
        {

            // DebugPin.ctl(Debug_PB5, true);
            start_tick = SrvOsCommon.get_systimer_current_tick();

            if(SrvIMU.sample(SrvIMU_FusModule))
            {
                end_tick = SrvOsCommon.get_systimer_current_tick();
                SrvIMU.error_proc();

                obj->statistic_imu->sample_cnt ++;
                obj->statistic_imu->nxt_sample_time = cur_time + sample_interval_ms;
                if(obj->statistic_imu->start_time == 0)
                    obj->statistic_imu->start_time = cur_time;

                state = true;
            }
            // DebugPin.ctl(Debug_PB5, false);

            if(state)
            {
                if(start_tick < end_tick)
                {
                    sample_tick = end_tick - start_tick;
                }
                else if(start_tick > end_tick)
                {
                    /* must after timer irq */
                    sample_tick = SrvOsCommon.get_systimer_period() - start_tick + end_tick;
                }

                if((sample_tick <= obj->statistic_imu->min_sampling_overhead) || \
                   (obj->statistic_imu->min_sampling_overhead == 0))
                    obj->statistic_imu->min_sampling_overhead = sample_tick;

                if(sample_tick >= obj->statistic_imu->max_sampling_overhead)
                    obj->statistic_imu->max_sampling_overhead = sample_tick;

                obj->statistic_imu->avg_sampling_overhead += sample_tick;
                obj->statistic_imu->avg_sampling_overhead /= 2;

                obj->statistic_imu->cur_sampling_overhead = sample_tick;
            }
        }
    }

    return state;
}

static SrvIMU_UnionData_TypeDef SrvSensorMonitor_Get_IMUData(SrvSensorMonitorObj_TypeDef *obj)
{
    SrvIMU_UnionData_TypeDef imu_data_tmp;

    memset(&imu_data_tmp, 0, sizeof(SrvIMU_UnionData_TypeDef));

    if(obj && obj->enabled_reg.bit.imu && obj->init_state_reg.bit.imu && SrvIMU.get_data)
    {
        imu_data_tmp.data = SrvIMU.get_data(SrvIMU_FusModule);

        /*
         * adjust imu axis coordinate as 
         * x -----> forward
         * y -----> right
         * z -----> down 
         **/

        imu_data_tmp.data.flt_acc[Axis_X] *= -1;
        imu_data_tmp.data.flt_acc[Axis_Y] *= -1;

        imu_data_tmp.data.org_acc[Axis_X] *= -1;
        imu_data_tmp.data.org_acc[Axis_Y] *= -1;

        imu_data_tmp.data.flt_gyr[Axis_X] *= -1;
        imu_data_tmp.data.flt_gyr[Axis_Y] *= -1;

        imu_data_tmp.data.org_gyr[Axis_X] *= -1;
        imu_data_tmp.data.org_gyr[Axis_Y] *= -1;

        for (uint8_t chk = 0; chk < sizeof(SrvIMU_UnionData_TypeDef) - sizeof(uint16_t); chk++)
        {
            imu_data_tmp.data.chk_sum += imu_data_tmp.buff[chk];
        }
    }

    return imu_data_tmp;
}

/******************************************* Mag Section **********************************************/
/* still in developing */
static bool SrvSensorMonitor_Mag_Init(void)
{
    return false;
}

static bool SrvSensorMonitor_Mag_SampleCTL(SrvSensorMonitorObj_TypeDef *obj)
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
static bool SrvSensorMonitor_Baro_Init(void)
{
    if(SrvBaro.init && (SrvBaro.init() == SrvBaro_Error_None))
        return true;

    return false;
}

static bool SrvSensorMonitor_Baro_SampleCTL(SrvSensorMonitorObj_TypeDef *obj)
{
    bool state = false;
    uint32_t sample_interval_ms = 0;
    uint32_t cur_time = SrvOsCommon.get_os_ms();
    uint32_t start_tick = 0;
    uint32_t end_tick = 0;
    int32_t sample_tick = 0;

    if(obj && obj->enabled_reg.bit.baro && obj->init_state_reg.bit.baro && obj->freq_reg.bit.baro && obj->statistic_baro->set_period)
    {
        sample_interval_ms = SrvSensorMonitor_GetSampleInterval(obj->statistic_baro->set_period);
    
        if( SrvBaro.sample && \ 
            ((obj->statistic_baro->start_time == 0) || \
             (sample_interval_ms && \
             (cur_time >= obj->statistic_baro->nxt_sample_time))))
        {
            start_tick = SrvOsCommon.get_systimer_current_tick();

            DebugPin.ctl(Debug_PB5, true);
            if(SrvBaro.sample())
            {
                end_tick = SrvOsCommon.get_systimer_current_tick();

                obj->statistic_baro->sample_cnt ++;
                obj->statistic_baro->nxt_sample_time = cur_time + sample_interval_ms;
                if(obj->statistic_baro->start_time == 0)
                    obj->statistic_baro->start_time = cur_time;

                state = true;
            }
            DebugPin.ctl(Debug_PB5, false);
        
            if(state)
            {
                if(start_tick < end_tick)
                {
                    sample_tick = end_tick - start_tick;
                }
                else if(start_tick > end_tick)
                {
                    /* must after timer irq */
                    sample_tick = SrvOsCommon.get_systimer_period() - start_tick + end_tick;
                }

                if((sample_tick <= obj->statistic_baro->min_sampling_overhead) || \
                    (obj->statistic_baro->min_sampling_overhead == 0))
                    obj->statistic_baro->min_sampling_overhead = sample_tick;

                if(sample_tick >= obj->statistic_baro->max_sampling_overhead)
                    obj->statistic_baro->max_sampling_overhead = sample_tick;

                obj->statistic_baro->avg_sampling_overhead += sample_tick;
                obj->statistic_baro->avg_sampling_overhead /= 2;

                obj->statistic_baro->cur_sampling_overhead = sample_tick;
            }
        }
    }

    return false;
}

static SrvBaroData_TypeDef SrvSensorMonitor_Get_BaroData(SrvSensorMonitorObj_TypeDef *obj)
{
    SrvBaroData_TypeDef baro_data_tmp;

    memset(&baro_data_tmp, 0, sizeof(SrvBaroData_TypeDef));

    if(obj && obj->enabled_reg.bit.baro && obj->init_state_reg.bit.baro && SrvBaro.get_data)
    {
        baro_data_tmp = SrvBaro.get_data();
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

static bool SrvSensorMonitor_Tof_SampleCTL(SrvSensorMonitorObj_TypeDef *obj)
{
    if(obj && obj->enabled_reg.bit.gnss && obj->init_state_reg.bit.gnss)
    {
        
    }

    return false;
}

/* noticed all sensor sampling and processing must finished in 1ms maximum */
/* maximum sampling rate is 1KHz currentlly */
static bool SrvSensorMonitor_SampleCTL(SrvSensorMonitorObj_TypeDef *obj)
{
    bool state = false;
    SrvIMU_Data_TypeDef pri_imu;
    SrvIMU_Data_TypeDef sec_imu;
    
    /* single sampling overhead is about 60us */

    state |= SrvSensorMonitor_IMU_SampleCTL(obj);
    
    if((obj->statistic_imu->is_calid == Sensor_Calib_Start) || \
       (obj->statistic_imu->is_calid == Sensor_Calib_InProcess))
    {
        memset(&pri_imu, 0, sizeof(SrvIMU_Data_TypeDef));
        memset(&sec_imu, 0, sizeof(SrvIMU_Data_TypeDef));

        pri_imu = SrvIMU.get_data(SrvIMU_PriModule);
        sec_imu = SrvIMU.get_data(SrvIMU_SecModule);

        switch((uint8_t) SrvIMU.calib(GYRO_CALIB_CYCLE, pri_imu.org_gyr, sec_imu.org_gyr))
        {
            case SrvIMU_Gyr_CalibFailed:
                obj->statistic_imu->is_calid = Sensor_Calib_Failed;
                break;

            case SrvIMU_Gyr_CalibDone:
                obj->statistic_imu->is_calid = Sensor_Calib_Success;
                break;

            case SrvIMU_Gyr_Calibarting:
                obj->statistic_imu->is_calid = Sensor_Calib_InProcess;
                break;

            default:
                obj->statistic_imu->is_calid = Sensor_Calib_Failed;
                break;
        }
    }
    
    state |= SrvSensorMonitor_Mag_SampleCTL(obj);
    
    if((obj->statistic_mag->is_calid == Sensor_Calib_Start) || \
       (obj->statistic_mag->is_calid == Sensor_Calib_InProcess))
    {

    }

    state |= SrvSensorMonitor_Baro_SampleCTL(obj);
    
    if((obj->statistic_baro->is_calid == Sensor_Calib_Start) || \
       (obj->statistic_baro->is_calid == Sensor_Calib_InProcess))
    {

    }

    state |= SrvSensorMonitor_Tof_SampleCTL(obj);
    
    if((obj->statistic_tof->is_calid == Sensor_Calib_Start) || \
       (obj->statistic_tof->is_calid == Sensor_Calib_InProcess))
    {
        
    }

    state |= SrvSensorMonitor_Gnss_SampleCTL(obj);
    
    return state;
}

static SrvSensorMonitor_CaliState_List SrvSensorMonitor_Set_Module_Calib(SrvSensorMonitorObj_TypeDef *obj, SrvSensorMonitor_Type_List type)
{
    if(obj)
    {
        switch((uint8_t) type)
        {
            case SrvSensorMonitor_Type_IMU:
                obj->statistic_imu->is_calid = Sensor_Calib_Start;
                break;

            case SrvSensorMonitor_Type_BARO:
                obj->statistic_baro->is_calid = Sensor_Calib_Start;
                break;

            case SrvSensorMonitor_Type_MAG:
                obj->statistic_mag->is_calid = Sensor_Calib_Start;
                break;

            case SrvSensorMonitor_Type_TOF:
                obj->statistic_tof->is_calid = Sensor_Calib_Start;
                break;

            default:
                return Sensor_Calib_Failed;
        }
    }

    return Sensor_Calib_Start;
}

static SrvSensorMonitor_CaliState_List SrvSensorMonitor_Get_Module_Calib(SrvSensorMonitorObj_TypeDef *obj, SrvSensorMonitor_Type_List type)
{
    if(obj)
    {
        switch((uint8_t) type)
        {
            case SrvSensorMonitor_Type_IMU:
                break;

            case SrvSensorMonitor_Type_MAG:
                if(obj->enabled_reg.bit.mag && obj->init_state_reg.bit.mag)
                {
                    return obj->statistic_mag->is_calid;
                }
                else
                    return Sensor_Calib_Failed; 

            case SrvSensorMonitor_Type_BARO:
                if(obj->enabled_reg.bit.baro && obj->init_state_reg.bit.baro)
                {
                    return obj->statistic_baro->is_calid;
                }
                else
                    return Sensor_Calib_Failed;

            case SrvSensorMonitor_Type_TOF:
                if(obj->enabled_reg.bit.tof && obj->init_state_reg.bit.tof)
                {
                    return obj->statistic_tof->is_calid;
                }
                else
                    return Sensor_Calib_Failed;

            case SrvSensorMonitor_Type_GNSS:
                if(obj->enabled_reg.bit.gnss && obj->init_state_reg.bit.gnss)
                {
                    return obj->statistic_gnss->is_calid;
                }
                else
                    return Sensor_Calib_Failed;

            default:
                return Sensor_Calib_Failed;
        }
    }

    return Sensor_Calib_Failed;
}
