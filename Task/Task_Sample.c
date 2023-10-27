#include "Task_Sample.h"
#include "debug_util.h"
#include "Srv_OsCommon.h"
#include "IO_Definition.h"
#include "debug_util.h"
#include "error_log.h"
#include "../System/DataPipe/DataPipe.h"

#define DATAPIPE_TRANS_TIMEOUT_100Ms 100

/* internal var */
static Error_Handler TaskInertial_ErrorLog_Handle = NULL;
static uint32_t TaskSample_Period = 0;
static Task_SensorMonitor_TypeDef TaskSensor_Monitor;
DataPipe_CreateDataObj(SrvIMU_UnionData_TypeDef, IMU_Data);

/* internal function */
static void TaskInertical_Blink_Notification(uint16_t duration);
static void TaskInertical_Led_Control(bool state);
static bool TaskSampleCtl_IMU(void);
static bool TaskSampleCtl_BARO(void);

/* external function */

void TaskSample_Init(uint32_t period, uint32_t sensor_enable)
{
    memset(&IMU_Smp_DataPipe, 0, sizeof(IMU_Smp_DataPipe));
    memset(DataPipe_DataObjAddr(IMU_Data), 0, sizeof(DataPipe_DataObj(IMU_Data)));
    memset(&TaskSensor_Monitor, 0, sizeof(TaskSensor_Monitor));

    TaskSensor_Monitor.enabled_reg = sensor_enable;
    IMU_Smp_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(IMU_Data);
    IMU_Smp_DataPipe.data_size = sizeof(DataPipe_DataObj(IMU_Data));
    DataPipe_Enable(&IMU_Smp_DataPipe);

    if(TaskSensor_Monitor.enabled_reg & Task_SensorField_IMU)
    {
        TaskSensor_Monitor.enable_num ++;

        if (SrvIMU.init() != SrvIMU_AllModule_Init_Error)
        {
            TaskSensor_Monitor.init_state_reg |= Task_SensorField_IMU;
        }
    }

    if(TaskSensor_Monitor.enabled_reg & Task_SensorField_BARO)
    {
        TaskSensor_Monitor.enable_num ++;

        if(SrvBaro.init() == SrvBaro_Error_None)
        {
            TaskSensor_Monitor.init_state_reg |= Task_SensorField_BARO;
        }
    }

    if(TaskSensor_Monitor.enable_num)
    {
        TaskSensor_Monitor.statistic_tab = SrvOsCommon.malloc(sizeof(Task_SensoSampleStatistic_TypeDef) * TaskSensor_Monitor.enable_num);

        if(TaskSensor_Monitor.statistic_tab == NULL)
        {
            SrvOsCommon.free(TaskSensor_Monitor.statistic_tab);
        }
    }
    else
        return;
        
    /* need pipe sensor state to datahub after initial */

    TaskSample_Period = period;
}

void TaskSample_Core(void const *arg)
{
    uint32_t sys_time = SrvOsCommon.get_os_ms();

    while(1)
    {
        TaskInertical_Blink_Notification(100);

        TaskSampleCtl_IMU();
        TaskSampleCtl_BARO();
        
        SrvOsCommon.precise_delay(&sys_time, TaskSample_Period);
    }
}

static bool TaskSampleCtl_IMU(void)
{

    if((TaskSensor_Monitor.enabled_reg & Task_SensorField_IMU) && (TaskSensor_Monitor.init_state_reg & Task_SensorField_IMU))
    {
        if ((SrvIMU.sample != NULL) && (SrvIMU.error_proc != NULL) && SrvIMU.sample(SrvIMU_Both_Sample))
        {
            DataPipe_DataObj(IMU_Data).data = SrvIMU.get_data(SrvIMU_FusModule);

            // DebugPin.ctl(Debug_PB5, true);
            
            for (uint8_t chk = 0; chk < sizeof(DataPipe_DataObj(IMU_Data)) - sizeof(uint16_t); chk++)
            {
                DataPipe_DataObj(IMU_Data).data.chk_sum += DataPipe_DataObj(IMU_Data).buff[chk];
            }

            /* need measurement the overhead from pipe send to pipe receive callback triggered */
            DataPipe_SendTo(&IMU_Smp_DataPipe, &IMU_Log_DataPipe); /* to Log task */
            DataPipe_SendTo(&IMU_Smp_DataPipe, &IMU_hub_DataPipe); /* to control task */

            SrvIMU.error_proc();

            // DebugPin.ctl(Debug_PB5, false);
            return true;
        }
    }

    return false;
}

static bool TaskSampleCtl_BARO(void)
{
    if((TaskSensor_Monitor.enabled_reg & Task_SensorField_BARO) && (TaskSensor_Monitor.init_state_reg & Task_SensorField_BARO))
    {
        /* sample baro sensor */
        if((TaskSensor_Monitor.enabled_reg & Task_SensorField_BARO) && (TaskSensor_Monitor.init_state_reg & Task_SensorField_BARO))
        {
            if(SrvBaro.sample != NULL)
            {
                return true;
            }
        }
    }

    return false;
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

static void TaskInertical_Led_Control(bool state)
{
    // DevLED.ctl(Led2, state);
}
