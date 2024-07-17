#include "Task_Sample.h"
#include "debug_util.h"
#include "Srv_OsCommon.h"
#include "HW_Def.h"
#include "error_log.h"
#include "../FCHW_Config.h"
#include "../System/DataPipe/DataPipe.h"
#include "Srv_SensorMonitor.h"

#define DATAPIPE_TRANS_TIMEOUT_100Ms 100

#if defined MATEKH743_V1_5
#define Sample_Blinkly Led2
#elif defined BATEAT32F435_AIO
#define Sample_Blinkly Led1
#endif

/* internal var */
static uint32_t TaskSample_Period = 0;
static bool sample_enable = false;
static SrvSensorMonitorObj_TypeDef SensorMonitor;
DataPipe_CreateDataObj(SrvIMU_UnionData_TypeDef, IMU_Data);
DataPipe_CreateDataObj(SrvBaro_UnionData_TypeDef, Baro_Data);
DataPipe_CreateDataObj(SrvSensorMonitor_GenReg_TypeDef, SensorEnable_State);
DataPipe_CreateDataObj(SrvSensorMonitor_GenReg_TypeDef, SensorInit_State);
DataPipe_CreateDataObj(SrvIMU_Range_TypeDef, Smp_PriIMU_Range);
#if (IMUJ_CNT == 2)
DataPipe_CreateDataObj(SrvIMU_Range_TypeDef, Smp_SecIMU_Range);
#endif

/* internal function */
static void TaskInertical_Blink_Notification(uint16_t duration);

/* external function */

void TaskSample_Init(uint32_t period)
{
    SrvSensorMonitor_IMURange_TypeDef PriIMU_Range;
    SrvSensorMonitor_IMURange_TypeDef SecIMU_Range;

    memset(&PriIMU_Range, 0, sizeof(SrvSensorMonitor_IMURange_TypeDef));
    memset(&SecIMU_Range, 0, sizeof(SrvSensorMonitor_IMURange_TypeDef));

    memset(&SensorMonitor, 0, sizeof(SrvSensorMonitorObj_TypeDef));
    memset(&IMU_Smp_DataPipe, 0, sizeof(IMU_Smp_DataPipe));
    memset(&Baro_smp_DataPipe, 0, sizeof(Baro_smp_DataPipe));

    memset(DataPipe_DataObjAddr(Baro_Data), 0, DataPipe_DataSize(Baro_Data));
    memset(DataPipe_DataObjAddr(IMU_Data), 0, DataPipe_DataSize(IMU_Data));

    IMU_Smp_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(IMU_Data);
    IMU_Smp_DataPipe.data_size = DataPipe_DataSize(IMU_Data);
    DataPipe_Enable(&IMU_Smp_DataPipe);
    
    Baro_smp_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Baro_Data);
    Baro_smp_DataPipe.data_size = DataPipe_DataSize(Baro_Data);
    DataPipe_Enable(&Baro_smp_DataPipe);

    SensorMonitor.enabled_reg.bit.imu = true;
    SensorMonitor.freq_reg.bit.imu = SrvSensorMonitor_SampleFreq_1KHz;
    
#if (BARO_SUM >= 1)
    SensorMonitor.enabled_reg.bit.baro = true;
    SensorMonitor.freq_reg.bit.baro = SrvSensorMonitor_SampleFreq_50Hz;
#endif

    sample_enable = SrvSensorMonitor.init(&SensorMonitor);

    DataPipe_DataObj(SensorEnable_State).val = SensorMonitor.enabled_reg.val;
    DataPipe_DataObj(SensorInit_State).val = SensorMonitor.init_state_reg.val;
    
    SensorEnableState_smp_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(SensorEnable_State);
    SensorEnableState_smp_DataPipe.data_size = DataPipe_DataSize(SensorEnable_State);
    DataPipe_Enable(&SensorEnableState_smp_DataPipe);
    
    SensorInitState_smp_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(SensorInit_State);
    SensorInitState_smp_DataPipe.data_size = DataPipe_DataSize(SensorInit_State);
    DataPipe_Enable(&SensorInitState_smp_DataPipe);

    /* pipe sensor enable and initial state to datahub */
    DataPipe_SendTo(&SensorEnableState_smp_DataPipe, &SensorEnableState_hub_DataPipe);
    DataPipe_SendTo(&SensorInitState_smp_DataPipe, &SensorInitState_hub_DataPipe);

    if(sample_enable)
    {
        if(SrvSensorMonitor.get_imu_range(&SensorMonitor, SrvIMU_PriModule, &PriIMU_Range))
        {
            DataPipe_DataObj(Smp_PriIMU_Range).Acc = PriIMU_Range.Acc;
            DataPipe_DataObj(Smp_PriIMU_Range).Gyr = PriIMU_Range.Gyr;

            IMU_PriRange_Smp_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Smp_PriIMU_Range);
            IMU_PriRange_Smp_DataPipe.data_size = DataPipe_DataSize(Smp_PriIMU_Range);
            DataPipe_Enable(&IMU_PriRange_Smp_DataPipe);
            
            /* pipe sensor sample range to datahub */
            DataPipe_SendTo(&IMU_PriRange_Smp_DataPipe, &IMU_PriRange_hub_DataPipe);
        }

#if (IMU_CNT == 2)
        if(SrvSensorMonitor.get_imu_range(&SensorMonitor, SrvIMU_SecModule, &SecIMU_Range))
        {
            DataPipe_DataObj(Smp_SecIMU_Range).Acc = SecIMU_Range.Acc;
            DataPipe_DataObj(Smp_SecIMU_Range).Gyr = SecIMU_Range.Gyr;
            
            IMU_SecRange_Smp_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Smp_SecIMU_Range);
            IMU_SecRange_Smp_DataPipe.data_size = DataPipe_DataSize(Smp_SecIMU_Range);
            DataPipe_Enable(&IMU_SecRange_Smp_DataPipe);
            
            /* pipe sensor sample range to datahub */
            DataPipe_SendTo(&IMU_SecRange_Smp_DataPipe, &IMU_SecRange_hub_DataPipe);
        }
#endif
    }

    /* force make sensor sample task run as 1khz freq */
    TaskSample_Period = 1;
}

void TaskSample_Core(void const *arg)
{
    uint32_t sys_time = SrvOsCommon.get_os_ms();
    
    while(1)
    {
        TaskInertical_Blink_Notification(100);

        if(sample_enable && SrvSensorMonitor.sample_ctl(&SensorMonitor))
        {
            DataPipe_DataObj(IMU_Data) = SrvSensorMonitor.get_imu_data(&SensorMonitor);
            DataPipe_DataObj(Baro_Data).data = SrvSensorMonitor.get_baro_data(&SensorMonitor);

            /* need measurement the overhead from pipe send to pipe receive callback triggered */
            // DebugPin.ctl(Debug_PB4, true);
            /* to Log task */
            DataPipe_SendTo(&IMU_Smp_DataPipe, &IMU_Log_DataPipe);
            DataPipe_SendTo(&Baro_smp_DataPipe, &Baro_Log_DataPipe);

            /* to data hub */
            DataPipe_SendTo(&IMU_Smp_DataPipe, &IMU_hub_DataPipe);
            DataPipe_SendTo(&Baro_smp_DataPipe, &Baro_hub_DataPipe);
            // DebugPin.ctl(Debug_PB4, false);
        }
        
        SrvOsCommon.precise_delay(&sys_time, TaskSample_Period);
    }
}

static void TaskInertical_Blink_Notification(uint16_t duration)
{
    uint32_t Rt = 0;
    static uint32_t Lst_Rt = 0;
    static bool led_state = false;

    Rt = SrvOsCommon.get_os_ms();

    if ((Rt % duration == 0) && (Lst_Rt != Rt))
    {
        led_state = !led_state;
        Lst_Rt = Rt;
    }

    DevLED.ctl(Sample_Blinkly, led_state);
}
