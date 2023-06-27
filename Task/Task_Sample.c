#include "Task_Sample.h"
#include "debug_util.h"
#include "Srv_OsCommon.h"
#include "IO_Definition.h"
#include "debug_util.h"
#include "error_log.h"
#include "Srv_OsCommon.h"
#include "../System/DataPipe/DataPipe.h"

#define DATAPIPE_TRANS_TIMEOUT_100Ms 100

/* internal var */
static Task_SensorInertial_State TaskInertial_State = Task_SensorInertial_Core;
static Error_Handler TaskInertial_ErrorLog_Handle = NULL;
static uint32_t TaskSample_Period = 0;
DataPipe_CreateDataObj(SrvIMU_UnionData_TypeDef, IMU_Data);

/* internal function */
static void TaskInertical_Blink_Notification(uint16_t duration);
static void TaskInertical_Led_Control(bool state);

/* external function */

void TaskSample_Init(uint32_t period)
{
    memset(&IMU_Smp_DataPipe, NULL, sizeof(IMU_Smp_DataPipe));
    memset(DataPipe_DataObjAddr(IMU_Data), NULL, sizeof(DataPipe_DataObj(IMU_Data)));

    IMU_Smp_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(IMU_Data);
    IMU_Smp_DataPipe.data_size = sizeof(DataPipe_DataObj(IMU_Data));
    DataPipe_Enable(&IMU_Smp_DataPipe);

    /* regist error */
    if (SrvIMU.init() == SrvIMU_AllModule_Init_Error)
        TaskInertial_State = Task_SensorInertial_Error;

    TaskSample_Period = period;
}

void TaskSample_Core(void const *arg)
{
    uint32_t sys_time = SrvOsCommon.get_os_ms();

    while(1)
    {
        // DebugPin.ctl(Debug_PB5, true);
        switch ((uint8_t)TaskInertial_State)
        {
        case Task_SensorInertial_Core:
            TaskInertical_Blink_Notification(100);
            if (SrvIMU.sample(SrvIMU_Both_Sample))
            {
                DataPipe_DataObj(IMU_Data).data = SrvIMU.get_data(SrvIMU_FusModule);

                for (uint8_t chk = 0; chk < sizeof(DataPipe_DataObj(IMU_Data)) - sizeof(uint16_t); chk++)
                {
                    DataPipe_DataObj(IMU_Data).data.chk_sum += DataPipe_DataObj(IMU_Data).buff[chk];
                }

                DataPipe_SendTo(&IMU_Smp_DataPipe, &IMU_Log_DataPipe); /* to Log task */
                DataPipe_SendTo(&IMU_Smp_DataPipe, &IMU_hub_DataPipe); /* to control task */

                if(DataPipe_DataObj(IMU_Data).data.error_code == SrvIMU_Sample_Over_Angular_Accelerate)
                {
                    TaskInertical_Led_Control(true);
                }
                else
                    TaskInertical_Led_Control(false);
            }
            break;

        case Task_SensorInertial_Error:
            break;

        default:
            break;
        }

        SrvIMU.error_proc();
        // DebugPin.ctl(Debug_PB5, false);

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

static void TaskInertical_Led_Control(bool state)
{
    // DevLED.ctl(Led2, state);
}
