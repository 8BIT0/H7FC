#include "Srv_SensorMonitor.h"
#include "Bsp_Timer.h"
#include "Srv_OsCommon.h"
#include "debug_util.h"
#include "HW_Def.h"

/* 
 * for sensor statistic function a timer is essenial
 */
#define SrvSensorMonitor_GetSampleInterval(period) (1000 / period)

#define FLOW_TYPE Flow_3901U

#define MONITOR_TAG "[ SENSOR MONITOR INFO ] "
#define MONITOR_INFO(fmt, ...) Debug_Print(&DebugPort, MONITOR_TAG, fmt, ##__VA_ARGS__)

/* internal function */
static uint32_t SrvSensorMonitor_Get_FreqVal(uint8_t freq_enum);

static bool SrvSensorMonitor_IMU_Init(SrvSensorMonitorObj_TypeDef *obj);
static bool SrvSensorMonitor_Mag_Init(void);
static bool SrvSensorMonitor_Baro_Init(SrvSensorMonitorObj_TypeDef *obj);
static bool SrvSensorMonitor_Flow_Init(void);

/* external function */
static bool SrvSensorMonitor_Init(SrvSensorMonitorObj_TypeDef *obj);
static bool SrvSensorMonitor_SampleCTL(SrvSensorMonitorObj_TypeDef *obj);
static SrvIMU_UnionData_TypeDef SrvSensorMonitor_Get_IMUData(SrvSensorMonitorObj_TypeDef *obj);
static GenCalib_State_TypeList SrvSensorMonitor_Set_Module_Calib(SrvSensorMonitorObj_TypeDef *obj, SrvSensorMonitor_Type_List type);
static GenCalib_State_TypeList SrvSensorMonitor_Get_Module_Calib(SrvSensorMonitorObj_TypeDef *obj, SrvSensorMonitor_Type_List type);
static SrvBaro_UnionData_TypeDef SrvSensorMonitor_Get_BaroData(SrvSensorMonitorObj_TypeDef *obj);
static bool SrvSensorMonitor_IMU_Get_Num(SrvSensorMonitorObj_TypeDef *obj, uint8_t *num);
static bool SrvSensorMonitor_Get_IMU_Range(SrvSensorMonitorObj_TypeDef *obj, SrvIMU_Module_Type type, SrvIMU_Range_TypeDef *range);

SrvSensorMonitor_TypeDef SrvSensorMonitor = {
    .init = SrvSensorMonitor_Init,
    .sample_ctl = SrvSensorMonitor_SampleCTL,
    .get_imu_num = SrvSensorMonitor_IMU_Get_Num,
    .get_imu_range = SrvSensorMonitor_Get_IMU_Range,
    .get_imu_data = SrvSensorMonitor_Get_IMUData,
    .get_baro_data = SrvSensorMonitor_Get_BaroData,
    .set_calib = SrvSensorMonitor_Set_Module_Calib,
    .get_calib = SrvSensorMonitor_Get_Module_Calib,
};

static bool SrvSensorMonitor_Init(SrvSensorMonitorObj_TypeDef *obj)
{
    uint8_t enable_sensor_num = 0;
    uint16_t list_index = 0;

    if (obj)
    {
        enable_sensor_num = Get_OnSet_Bit_Num(obj->enabled_reg.val);
        
        obj->statistic_list = SrvOsCommon.malloc(enable_sensor_num * sizeof(SrvSensorMonitor_Statistic_TypeDef));
        if (obj->statistic_list == NULL)
        {
            SrvOsCommon.free(obj->statistic_list);
            return false;
        }

        /* enabled on imu must be essential */
        if (obj->enabled_reg.bit.imu && SrvSensorMonitor_IMU_Init(obj))
        {
            if (list_index > enable_sensor_num)
                return false;

            obj->init_state_reg.bit.imu = true;
            obj->statistic_imu = &obj->statistic_list[list_index];
            obj->statistic_imu->set_period = SrvSensorMonitor_Get_FreqVal(obj->freq_reg.bit.imu);
            obj->statistic_imu->is_calid = Calib_None; 
            list_index ++;
        }
        else
        {
            if (obj->enabled_reg.bit.imu)
                obj->init_state_reg.bit.imu = false;

            return false;
        }

        if (obj->enabled_reg.bit.mag && SrvSensorMonitor_Mag_Init())
        {
            if (list_index > enable_sensor_num)
                return false;

            obj->init_state_reg.bit.mag = true;
            obj->statistic_mag = &obj->statistic_list[list_index];
            obj->statistic_mag->set_period = SrvSensorMonitor_Get_FreqVal(obj->freq_reg.bit.mag);
            obj->statistic_mag->is_calid = Calib_None;
            list_index ++;
        }
        else
        {
            obj->statistic_mag->is_calid = Calib_None;
            obj->init_state_reg.bit.mag = false;
        }

        if (obj->enabled_reg.bit.baro && SrvSensorMonitor_Baro_Init(obj))
        {
            if (list_index > enable_sensor_num)
                return false;

            obj->init_state_reg.bit.baro = true;
            obj->statistic_baro = &obj->statistic_list[list_index];
            obj->statistic_baro->set_period = SrvSensorMonitor_Get_FreqVal(obj->freq_reg.bit.baro);
            obj->statistic_baro->is_calid = Calib_None;
            list_index ++;
        }
        else
        {
            obj->statistic_baro->is_calid = Calib_None;
            obj->init_state_reg.bit.baro = false;
        }

        if (obj->enabled_reg.bit.flow && SrvSensorMonitor_Flow_Init())
        {
            if (list_index > enable_sensor_num)
                return false;

            obj->init_state_reg.bit.flow = true;
            obj->statistic_flow = &obj->statistic_list[list_index];
            obj->statistic_flow->set_period = SrvSensorMonitor_Get_FreqVal(obj->freq_reg.bit.flow);
            list_index ++;
        }
        else
        {
            obj->init_state_reg.bit.flow = false;
        }

        return true;
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
    if (obj)
        return obj->enabled_reg.val;

    return 0;
}

static uint32_t SrvSensorMonitor_Get_InitState(SrvSensorMonitorObj_TypeDef *obj)
{
    if (obj)
        return obj->init_state_reg.val;

    return 0;
}

/******************************************* IMU Section **********************************************/
static bool SrvSensorMonitor_IMU_Init(SrvSensorMonitorObj_TypeDef *obj)
{
    SrvIMU_ErrorCode_List init_state;

    if (SrvIMU.init && obj)
    {
        init_state = SrvIMU.init();
        obj->imu_num = 0;

        switch(init_state)
        {
            case SrvIMU_PriDev_Init_Error:
                MONITOR_INFO("Pri IMU init error\r\n");
                memset(&obj->PriIMU_Range, 0, sizeof(obj->PriIMU_Range));
                obj->sec_range_get = SrvIMU.get_range(SrvIMU_SecModule, &obj->SecIMU_Range);
                if (!obj->sec_range_get)
                {
                    MONITOR_INFO("IMU get range failed\r\n");
                    memset(&obj->SecIMU_Range, 0, sizeof(obj->SecIMU_Range));
                    return false;
                }

                obj->imu_num = 1;
                /* set sample mode */
                obj->IMU_SampleMode = SrvIMU_Priori_Sec;

                /* get secondary imu type */
                SrvIMU.get_type(SrvIMU_SecModule, &obj->sec_imu_type);
                obj->pri_imu_type = SrvIMU_Dev_None;
                break;

            case SrvIMU_SecDev_Init_Error:
                MONITOR_INFO("Sec IMU init error\r\n");
                memset(&obj->SecIMU_Range, 0, sizeof(obj->SecIMU_Range));
                obj->pri_range_get = SrvIMU.get_range(SrvIMU_PriModule, &obj->PriIMU_Range);
                if (!obj->pri_range_get)
                {
                    MONITOR_INFO("IMU get range failed\r\n");
                    memset(&obj->PriIMU_Range, 0, sizeof(obj->PriIMU_Range));
                    return false;
                }

                obj->imu_num = 1;
                /* ser sample mode */
                obj->IMU_SampleMode = SrvIMU_Priori_Pri;

                /* get primary imu type */
                SrvIMU.get_type(SrvIMU_PriModule, &obj->pri_imu_type);
                obj->sec_imu_type = SrvIMU_Dev_None;
                break;

            case SrvIMU_No_Error:
                MONITOR_INFO("All IMU init successed\r\n");
                memset(&obj->PriIMU_Range, 0, sizeof(obj->PriIMU_Range));
                memset(&obj->SecIMU_Range, 0, sizeof(obj->SecIMU_Range));
                
                obj->pri_range_get = SrvIMU.get_range(SrvIMU_PriModule, &obj->PriIMU_Range);
                obj->sec_range_get = SrvIMU.get_range(SrvIMU_SecModule, &obj->SecIMU_Range);
                
                /* get both imu type */
                SrvIMU.get_type(SrvIMU_PriModule, &obj->pri_imu_type);
                SrvIMU.get_type(SrvIMU_SecModule, &obj->sec_imu_type);

                /* set sample mode */
                if (!obj->pri_range_get && !obj->sec_range_get)
                {
                    MONITOR_INFO("IMU get range failed\r\n");
                    memset(&obj->PriIMU_Range, 0, sizeof(obj->PriIMU_Range));
                    memset(&obj->SecIMU_Range, 0, sizeof(obj->SecIMU_Range));
                    return false;
                }
                else
                {
                    if (obj->pri_range_get & obj->sec_range_get)
                    {
                        obj->imu_num = 2;
                        obj->IMU_SampleMode = SrvIMU_Both_Sample;
                    }
                    else
                    {
                        obj->imu_num = 1;
                        if (obj->pri_range_get)
                        {
                            obj->IMU_SampleMode = SrvIMU_Priori_Pri;
                        }
                        else
                            obj->IMU_SampleMode = SrvIMU_Priori_Sec;
                    }
                }
                break;

            case SrvIMU_AllModule_Init_Error:
            default:
                return false;
        }

        return true;
    }

    return false;
}

static bool SrvSensorMonitor_IMU_Get_Num(SrvSensorMonitorObj_TypeDef *obj, uint8_t *num)
{
    if (obj)
    {
        if (num)
        {
            if (obj->init_state_reg.bit.imu)
            {
                (*num) = obj->imu_num;
                return true;
            }
            else
                (*num) = 0;
        }   
    }

    return false;
}

static bool SrvSensorMonitor_Get_IMU_Range(SrvSensorMonitorObj_TypeDef *obj, SrvIMU_Module_Type type, SrvIMU_Range_TypeDef *range)
{
    if (obj && range)
    {
        switch((uint8_t)type)
        {
            case SrvIMU_PriModule:
                if (obj->pri_range_get)
                {
                    range->Acc = obj->PriIMU_Range.Acc;
                    range->Gyr = obj->PriIMU_Range.Gyr;
                    return true;
                }

                range->Acc = 0;
                range->Gyr = 0;
                return false;

            case SrvIMU_SecModule:
                if (obj->sec_range_get)
                {
                    range->Acc = obj->SecIMU_Range.Acc;
                    range->Gyr = obj->SecIMU_Range.Gyr;
                    return true;
                }

                range->Acc = 0;
                range->Gyr = 0;
                return false;

            default:
                return false;
        }
    }

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

    if (obj && obj->statistic_imu && obj->enabled_reg.bit.imu && obj->init_state_reg.bit.imu && obj->statistic_imu->set_period)
    {
        sample_interval_ms = SrvSensorMonitor_GetSampleInterval(obj->statistic_imu->set_period);
        
        if ( SrvIMU.sample && \
            SrvIMU.error_proc && 
            ((obj->statistic_imu->start_time == 0) || \
             (sample_interval_ms && \
             (cur_time >= obj->statistic_imu->nxt_sample_time))))
        {

            // DebugPin.ctl(Debug_PB5, true);
            start_tick = SrvOsCommon.get_systimer_current_tick();

            if (SrvIMU.sample(obj->IMU_SampleMode))
            { 
                end_tick = SrvOsCommon.get_systimer_current_tick();
                SrvIMU.error_proc();

                obj->statistic_imu->sample_cnt ++;
                obj->statistic_imu->nxt_sample_time = cur_time + sample_interval_ms;
                if (obj->statistic_imu->start_time == 0)
                    obj->statistic_imu->start_time = cur_time;

                state = true;
            }
            else
                end_tick = SrvOsCommon.get_systimer_current_tick();
            // DebugPin.ctl(Debug_PB5, false);

            if (state)
            {
                if (start_tick < end_tick)
                {
                    sample_tick = end_tick - start_tick;
                }
                else if (start_tick > end_tick)
                {
                    /* must after timer irq */
                    sample_tick = SrvOsCommon.get_systimer_period() - start_tick + end_tick;
                }

                if ((sample_tick <= obj->statistic_imu->min_sampling_overhead) || \
                   (obj->statistic_imu->min_sampling_overhead == 0))
                    obj->statistic_imu->min_sampling_overhead = sample_tick;

                if (sample_tick >= obj->statistic_imu->max_sampling_overhead)
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

    if (obj && obj->enabled_reg.bit.imu && obj->init_state_reg.bit.imu && SrvIMU.get_data)
    {
        if (SrvIMU.get_data(SrvIMU_FusModule, &imu_data_tmp.data))
        {
            /*
            * adjust imu axis coordinate as 
            * x -----> forward
            * y -----> right
            * z -----> down 
            **/
            for (uint8_t chk = 0; chk < sizeof(SrvIMU_UnionData_TypeDef) - sizeof(uint16_t); chk++)
            {
                imu_data_tmp.data.chk_sum += imu_data_tmp.buff[chk];
            }

            obj->lst_imu_data.data = imu_data_tmp.data;
        }
        else
            imu_data_tmp.data = obj->lst_imu_data.data;
    }
    else
        imu_data_tmp = obj->lst_imu_data;

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
    if (obj && obj->enabled_reg.bit.mag && obj->init_state_reg.bit.mag)
    {
        
    }

    return false;
}

static uint32_t SrvSensorMonitor_Get_MagData(SrvSensorMonitorObj_TypeDef *obj)
{
    return 0;
}

/******************************************* Baro Section *********************************************/
static bool SrvSensorMonitor_Baro_Init(SrvSensorMonitorObj_TypeDef *obj)
{
    if (SrvBaro.init && obj)
    {
        obj->baro_type = BARO_TYPE;
        obj->baro_bus_type = BARO_BUS_TYPE;
        obj->baro_err = SrvBaro.init(BARO_TYPE, BARO_BUS_TYPE);

        if (obj->baro_err == SrvBaro_Error_None)
        {
            MONITOR_INFO("Baro init accomplished\r\n");
            return true;
        }
        MONITOR_INFO("Baro init failed\r\n");
    }

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

    if (obj && obj->enabled_reg.bit.baro && obj->init_state_reg.bit.baro && obj->freq_reg.bit.baro && obj->statistic_baro->set_period)
    {
        sample_interval_ms = SrvSensorMonitor_GetSampleInterval(obj->statistic_baro->set_period);
    
        if (SrvBaro.sample && \
            ((obj->statistic_baro->start_time == 0) || \
             (sample_interval_ms && \
             (cur_time >= obj->statistic_baro->nxt_sample_time))))
        {
            start_tick = SrvOsCommon.get_systimer_current_tick();

            // DebugPin.ctl(Debug_PB5, true);
            if (SrvBaro.sample())
            {
                end_tick = SrvOsCommon.get_systimer_current_tick();

                obj->statistic_baro->sample_cnt ++;
                obj->statistic_baro->nxt_sample_time = cur_time + sample_interval_ms;
                if (obj->statistic_baro->start_time == 0)
                    obj->statistic_baro->start_time = cur_time;

                state = true;
            }
            else
                end_tick = SrvOsCommon.get_systimer_current_tick();
            // DebugPin.ctl(Debug_PB5, false);
        
            if (state)
            {
                if (start_tick < end_tick)
                {
                    sample_tick = end_tick - start_tick;
                }
                else if (start_tick > end_tick)
                {
                    /* must after timer irq */
                    sample_tick = SrvOsCommon.get_systimer_period() - start_tick + end_tick;
                }

                if ((sample_tick <= obj->statistic_baro->min_sampling_overhead) || \
                    (obj->statistic_baro->min_sampling_overhead == 0))
                    obj->statistic_baro->min_sampling_overhead = sample_tick;

                if (sample_tick >= obj->statistic_baro->max_sampling_overhead)
                    obj->statistic_baro->max_sampling_overhead = sample_tick;

                obj->statistic_baro->avg_sampling_overhead += sample_tick;
                obj->statistic_baro->avg_sampling_overhead /= 2;

                obj->statistic_baro->cur_sampling_overhead = sample_tick;
            }
        }
    }

    return state;
}

static SrvBaro_UnionData_TypeDef SrvSensorMonitor_Get_BaroData(SrvSensorMonitorObj_TypeDef *obj)
{
    SrvBaro_UnionData_TypeDef baro_data_tmp;

    memset(&baro_data_tmp, 0, sizeof(SrvBaro_UnionData_TypeDef));

    if (obj && obj->enabled_reg.bit.baro && obj->init_state_reg.bit.baro && SrvBaro.get_data)
    {
        if (SrvBaro.get_data(&baro_data_tmp))
        {
            obj->lst_baro_data.data = baro_data_tmp.data;
        }
        else
            baro_data_tmp.data = obj->lst_baro_data.data;
    }

    return baro_data_tmp;
}

/******************************************* Flow Section **********************************************/
static bool SrvSensorMonitor_Flow_Init(void)
{
    // return SrvFlow.init(FLOW_TYPE);
    return false;
}

static bool SrvSensorMonitor_Flow_SampleCTL(SrvSensorMonitorObj_TypeDef *obj)
{
    uint32_t sample_interval_ms = 0;
    uint32_t cur_time = 0;
    uint32_t start_tick = 0;
    uint32_t end_tick = 0;
    uint32_t sample_tick = 0;
    bool state = false;

    if (obj && obj->enabled_reg.bit.flow && obj->init_state_reg.bit.flow && obj->freq_reg.bit.flow && obj->statistic_flow->set_period)
    {
        sample_interval_ms = SrvSensorMonitor_GetSampleInterval(obj->statistic_flow->set_period);

        if (SrvFlow.sample && \
            ((obj->statistic_flow->start_time == 0) || \
             (sample_interval_ms && \
             (cur_time >= obj->statistic_baro->nxt_sample_time))))
        {
            start_tick = SrvOsCommon.get_systimer_current_tick();
            if (SrvFlow.sample(FLOW_TYPE))
            {
                end_tick = SrvOsCommon.get_systimer_current_tick();

                obj->statistic_baro->sample_cnt ++;
                obj->statistic_baro->nxt_sample_time = cur_time + sample_interval_ms;
                if (obj->statistic_baro->start_time == 0)
                    obj->statistic_baro->start_time = cur_time;

                state = true;
            }
            else
                end_tick = SrvOsCommon.get_systimer_current_tick();

            /* get flow sensor sample over head */
            if (state)
            {
                if (start_tick < end_tick)
                {
                    sample_tick = end_tick - start_tick;
                }
                else if (start_tick > end_tick)
                {
                    /* must after timer irq */
                    sample_tick = SrvOsCommon.get_systimer_period() - start_tick + end_tick;
                }

                if ((sample_tick <= obj->statistic_flow->min_sampling_overhead) || \
                   (obj->statistic_flow->min_sampling_overhead == 0))
                    obj->statistic_flow->min_sampling_overhead = sample_tick;

                if (sample_tick >= obj->statistic_flow->max_sampling_overhead)
                    obj->statistic_flow->max_sampling_overhead = sample_tick;

                obj->statistic_flow->avg_sampling_overhead += sample_tick;
                obj->statistic_flow->avg_sampling_overhead /= 2;

                obj->statistic_flow->cur_sampling_overhead = sample_tick;
            }
        }
    }

    return state;
}



/* noticed all sensor sampling and processing must finished in 1ms maximum */
/* maximum sampling rate is 1KHz currentlly */
static bool SrvSensorMonitor_SampleCTL(SrvSensorMonitorObj_TypeDef *obj)
{
    bool state = false;
    // DebugPin.ctl(Debug_PB5, true);
    
    /* imu single sampling overhead is about 60us */
    state |= SrvSensorMonitor_IMU_SampleCTL(obj);
    if (obj->statistic_imu->is_calid == Calib_Start)
    {
        if (SrvIMU.set_calib)
        {
            obj->statistic_imu->is_calid = SrvIMU.set_calib(GYRO_CALIB_CYCLE);
        }
        else
            obj->statistic_imu->is_calid = Calib_Failed;
    }
    else if (obj->statistic_imu->is_calid != Calib_Failed)
        obj->statistic_imu->is_calid = SrvIMU.get_calib();
    
    state |= SrvSensorMonitor_Mag_SampleCTL(obj);

    /* baro single sampling overhead is about 230us */
    state |= SrvSensorMonitor_Baro_SampleCTL(obj);
    if (obj->statistic_baro->is_calid == Calib_Start)
    {
        if (SrvBaro.set_calib)
        {
            obj->statistic_baro->is_calid = SrvBaro.set_calib(BARO_CALIB_CYCLE);
        }
        else
            obj->statistic_baro->is_calid = Calib_Failed;
    }
    else if (obj->statistic_baro->is_calid != Calib_Failed)
        obj->statistic_baro->is_calid = SrvBaro.get_calib();

    // DebugPin.ctl(Debug_PB5, false);
    
    return state;
}

static GenCalib_State_TypeList SrvSensorMonitor_Set_Module_Calib(SrvSensorMonitorObj_TypeDef *obj, SrvSensorMonitor_Type_List type)
{
    if (obj)
    {
        switch((uint8_t) type)
        {
            case SrvSensorMonitor_Type_IMU:
                obj->statistic_imu->is_calid = Calib_Start;
                break;

            case SrvSensorMonitor_Type_BARO:
                obj->statistic_baro->is_calid = Calib_Start;
                break;

            case SrvSensorMonitor_Type_MAG:
                obj->statistic_mag->is_calid = Calib_Start;
                break;

            default:
                return Calib_Start;
        }
    }

    return Calib_Start;
}

static GenCalib_State_TypeList SrvSensorMonitor_Get_Module_Calib(SrvSensorMonitorObj_TypeDef *obj, SrvSensorMonitor_Type_List type)
{
    if (obj)
    {
        switch((uint8_t) type)
        {
            case SrvSensorMonitor_Type_IMU:
                break;

            case SrvSensorMonitor_Type_MAG:
                if (obj->enabled_reg.bit.mag && obj->init_state_reg.bit.mag)
                {
                    return obj->statistic_mag->is_calid;
                }
                else
                    return Calib_Failed; 

            case SrvSensorMonitor_Type_BARO:
                if (obj->enabled_reg.bit.baro && obj->init_state_reg.bit.baro)
                {
                    return obj->statistic_baro->is_calid;
                }
                else
                    return Calib_Failed;

            default:
                return Calib_Failed;
        }
    }

    return Calib_Failed;
}


