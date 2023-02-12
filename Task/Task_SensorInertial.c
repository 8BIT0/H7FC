#include "Task_SensorInertial.h"
#include "scheduler.h"
#include "debug_util.h"
#include "runtime.h"
#include "IO_Definition.h"
#include "debug_util.h"
#include "error_log.h"
#include "DataPipe/DataPipe.h"

#define DATAPIPE_TRANS_TIMEOUT_100Ms 100

/* internal var */
static Task_SensorInertial_State TaskInertial_State = Task_SensorInertial_Core;
static Error_Handler TaskInertial_ErrorLog_Handle = NULL;

DataPipe_CreateDataObj(SrvIMU_UnionData_TypeDef, PriIMU_Data);
DataPipe_CreateDataObj(SrvIMU_UnionData_TypeDef, SecIMU_Data);

/* internal function */
static void TaskInertical_Blink_Notification(uint16_t duration);

/* external function */

void TaskInertial_Init(void)
{
    memset(&IMU_Smp_DataPipe, NULL, sizeof(IMU_Smp_DataPipe));
    memset(DataPipe_DataObjAddr(PriIMU_Data), NULL, sizeof(DataPipe_DataObj(PriIMU_Data)));
    memset(DataPipe_DataObjAddr(SecIMU_Data), NULL, sizeof(DataPipe_DataObj(SecIMU_Data)));

    IMU_Smp_DataPipe.data_addr = (uint32_t)&DataPipe_DataObj(PriIMU_Data);
    IMU_Smp_DataPipe.data_size = sizeof(DataPipe_DataObj(PriIMU_Data));

    /* regist error */
    if (SrvIMU.init() == SrvIMU_AllModule_Init_Error)
        TaskInertial_State = Task_SensorInertial_Error;
}

void TaskInertical_Core(Task_Handle hdl)
{
    DebugPin.ctl(Debug_PB5, true);
    switch ((uint8_t)TaskInertial_State)
    {
    case Task_SensorInertial_Core:
        // TaskInertical_Blink_Notification(100);
        if(SrvIMU.sample())
        {
            DataPipe_DataObj(PriIMU_Data).data = SrvIMU.get_data(SrvIMU_PriModule);
            DataPipe_DataObj(SecIMU_Data).data = SrvIMU.get_data(SrvIMU_SecModule);

            for(uint8_t chk = 0; chk < sizeof(DataPipe_DataObj(PriIMU_Data)) - sizeof(uint16_t); chk++)
            {
                DataPipe_DataObj(PriIMU_Data).data.chk_sum += DataPipe_DataObj(PriIMU_Data).buff[chk];
                DataPipe_DataObj(SecIMU_Data).data.chk_sum += DataPipe_DataObj(SecIMU_Data).buff[chk];
            }

            DataPipe_SendTo(&IMU_Smp_DataPipe, &IMU_Log_DataPipe);  /* to Log task */
            DataPipe_SendTo(&IMU_Smp_DataPipe, &IMU_Ctl_DataPipe);  /* to control task */
            // DataPipe_SendTo(&IMU_Smp_DataPipe, &IMU_Ptl_DataPipe);  /* to protocol task */
        }
        break;

    case Task_SensorInertial_Error:
        break;

    default:
        break;
    }

    SrvIMU.error_proc();
    DebugPin.ctl(Debug_PB5, false);
}

static void TaskInertical_Blink_Notification(uint16_t duration)
{
    SYSTEM_RunTime Rt = 0;
    static SYSTEM_RunTime Lst_Rt = 0;
    static bool led_state = false;

    Rt = Get_CurrentRunningMs();

    if ((Rt % duration == 0) && (Lst_Rt != Rt))
    {
        led_state = !led_state;
        Lst_Rt = Rt;
    }

    DevLED.ctl(Led2, led_state);
    // test_PC1_ctl();
}
