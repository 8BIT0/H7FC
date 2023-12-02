#include "Task_Sample.h"
#include "debug_util.h"
#include "Srv_OsCommon.h"
#include "IO_Definition.h"
#include "debug_util.h"
#include "error_log.h"
#include "../System/DataPipe/DataPipe.h"
#include "Srv_SensorMonitor.h"

#define DATAPIPE_TRANS_TIMEOUT_100Ms 100

/* internal var */
static Error_Handler TaskInertial_ErrorLog_Handle = NULL;
static uint32_t TaskSample_Period = 0;
static SrvSensorMonitorObj_TypeDef SensorMonitor;
DataPipe_CreateDataObj(SrvIMU_UnionData_TypeDef, IMU_Data);
DataPipe_CreateDataObj(SrvBaroData_TypeDef, Baro_Data);
DataPipe_CreateDataObj(SrvSensorMonitor_GenReg_TypeDef, SensorEnable_State);
DataPipe_CreateDataObj(SrvSensorMonitor_GenReg_TypeDef, SensorInit_State);

/* internal function */
static void TaskInertical_Blink_Notification(uint16_t duration);

/* external function */

void TaskSample_Init(uint32_t period)
{
    memset(&SensorMonitor, 0, sizeof(SrvSensorMonitorObj_TypeDef));
    memset(&IMU_Smp_DataPipe, 0, sizeof(IMU_Smp_DataPipe));
    memset(&Baro_smp_DataPipe, 0, sizeof(Baro_smp_DataPipe));

    memset(DataPipe_DataObjAddr(Baro_Data), 0, sizeof(DataPipe_DataObj(Baro_Data)));
    memset(DataPipe_DataObjAddr(IMU_Data), 0, sizeof(DataPipe_DataObj(IMU_Data)));

    IMU_Smp_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(IMU_Data);
    IMU_Smp_DataPipe.data_size = sizeof(DataPipe_DataObj(IMU_Data));
    DataPipe_Enable(&IMU_Smp_DataPipe);
    
    Baro_smp_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Baro_Data);
    Baro_smp_DataPipe.data_size = sizeof(DataPipe_DataObj(Baro_Data));
    DataPipe_Enable(&Baro_smp_DataPipe);

    SensorMonitor.enabled_reg.bit.imu = true;
    SensorMonitor.freq_reg.bit.imu = SrvSensorMonitor_SampleFreq_1KHz;
    
    SensorMonitor.enabled_reg.bit.baro = true;
    SensorMonitor.freq_reg.bit.baro = SrvSensorMonitor_SampleFreq_100Hz;

    SrvSensorMonitor.init(&SensorMonitor);

    DataPipe_DataObj(SensorEnable_State).val = SensorMonitor.enabled_reg.val;
    DataPipe_DataObj(SensorInit_State).val = SensorMonitor.init_state_reg.val;
    
    SensorEnableState_smp_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(SensorEnable_State);
    SensorEnableState_smp_DataPipe.data_size = sizeof(DataPipe_DataObj(SensorEnable_State));
    DataPipe_Enable(&SensorEnableState_smp_DataPipe);
    
    SensorInitState_smp_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(SensorInit_State);
    SensorInitState_smp_DataPipe.data_size = sizeof(DataPipe_DataObj(SensorInit_State));
    DataPipe_Enable(&SensorInitState_smp_DataPipe);

    /* need pipe sensor state to datahub after initial */
    DataPipe_SendTo(&SensorInitState_smp_DataPipe, &SensorInitState_hub_DataPipe);
    DataPipe_SendTo(&SensorEnableState_smp_DataPipe, &SensorEnableState_hub_DataPipe);

    /* force make sensor sample task run as 1khz freq */
    TaskSample_Period = 1;
}

void TaskSample_Core(void const *arg)
{
    uint32_t sys_time = SrvOsCommon.get_os_ms();
    
    while(1)
    {
        TaskInertical_Blink_Notification(100);

        if(SrvSensorMonitor.sample_ctl(&SensorMonitor))
        {
            DataPipe_DataObj(IMU_Data) = SrvSensorMonitor.get_imu_data(&SensorMonitor);
            DataPipe_DataObj(Baro_Data) = SrvSensorMonitor.get_baro_data(&SensorMonitor);

            /* need measurement the overhead from pipe send to pipe receive callback triggered */
            DataPipe_SendTo(&IMU_Smp_DataPipe, &IMU_Log_DataPipe); /* to Log task */
            DataPipe_SendTo(&IMU_Smp_DataPipe, &IMU_hub_DataPipe);
            DataPipe_SendTo(&Baro_smp_DataPipe, &Baro_hub_DataPipe);
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

    DevLED.ctl(Led2, led_state);
    // test_PC1_ctl();
}
