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
DataPipeObj_TypeDef IMU_Smp_DataPipe;
static Task_SensorInertial_State TaskInertial_State = Task_SensorInertial_Core;
static Error_Handler TaskInertial_ErrorLog_Handle = NULL;
static SrvIMU_UnionData_TypeDef PriIMU_Data __attribute__((section(".Perph_Section")));
static SrvIMU_UnionData_TypeDef SecIMU_Data __attribute__((section(".Perph_Section")));

/* internal function */
static void TaskInertical_Blink_Notification(uint16_t duration);

/* external function */

void TaskInertial_Init(void)
{
    memset(&IMU_Smp_DataPipe, NULL, sizeof(IMU_Smp_DataPipe));
    memset(&PriIMU_Data, NULL, sizeof(PriIMU_Data));
    memset(&SecIMU_Data, NULL, sizeof(SecIMU_Data));

    IMU_Smp_DataPipe.data_addr = (uint32_t)&PriIMU_Data;
    IMU_Smp_DataPipe.data_size = sizeof(PriIMU_Data);

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
            PriIMU_Data.data = SrvIMU.get_data(SrvIMU_PriModule);
            SecIMU_Data.data = SrvIMU.get_data(SrvIMU_SecModule);

            for(uint8_t chk = 0; chk < sizeof(PriIMU_Data) - sizeof(uint16_t); chk++)
            {
                PriIMU_Data.data.chk_sum += PriIMU_Data.buff[chk];
                SecIMU_Data.data.chk_sum += SecIMU_Data.buff[chk];
            }

            DataPipe_SendTo(&IMU_Smp_DataPipe, &IMU_Log_DataPipe);
            DataPipe_SendTo(&IMU_Smp_DataPipe, &IMU_Ptl_DataPipe);
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
