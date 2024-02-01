#include "Task_Manager.h"
#include "Task_Log.h"
#include "Task_Control.h"
#include "Task_Sample.h"
#include "Task_Telemetry.h"
#include "Task_Navi.h"
#include "Task_Protocol.h"
#include "debug_util.h"
#include "HW_Def.h"
#include "Dev_Led.h"
#include "DiskIO.h"
#include "Srv_ComProto.h"
#include "Srv_OsCommon.h"
#include "../DataPipe/DataPipe.h"
#include "shell_port.h"
#include "Storage.h"

#define TaskSample_Period_Def    1  /* unit: ms period 1ms  1000Hz */
#define TaskControl_Period_Def   5  /* unit: ms period 2ms  200Hz  */
#define TaskTelemetry_Period_def 2  /* unit: ms period 2ms  500Hz  */
#define TaslLog_Period_Def       5  /* unit: ms period 5ms  200Hz  */
#define TaslNavi_Period_Def      10 /* unit: ms period 10ms 100Hz  */
#define TaskFrameCTL_Period_Def  5  /* unit: ms period 5ms  200Hz  */

osThreadId TaskInertial_Handle = NULL;
osThreadId TaskControl_Handle = NULL;
osThreadId TaskNavi_Handle = NULL;
osThreadId TaskLog_Handle = NULL;
osThreadId TaskTelemetry_Handle = NULL;
osThreadId TaskFrameCTL_Handle = NULL;
osThreadId TaskManager_Handle = NULL;

void Task_Manager_Init(void)
{
#if defined MATEKH743_V1_5
    DevLED.init(Led1);
    DevLED.init(Led2);
    DevLED.init(Led3);

    DebugPin.init(Debug_PC0);
    DebugPin.init(Debug_PC1);
    DebugPin.init(Debug_PC2);
    DebugPin.init(Debug_PC3);
    DebugPin.init(Debug_PB3);
    DebugPin.init(Debug_PB4);
    DebugPin.init(Debug_PB5);
    DebugPin.init(Debug_PB6);
    DebugPin.init(Debug_PB10);
#endif
    /* vol ADC init */

    /* cur ADC init */

    osThreadDef(ManagerTask, Task_Manager_CreateTask, osPriorityLow, 0, 512);
    TaskManager_Handle = osThreadCreate(osThread(ManagerTask), NULL);

    osKernelStart();
}

void Task_Manager_CreateTask(void)
{
    bool init = false;
    uint32_t enabled_sensor = 0;
    Storage_ModuleState_TypeDef storage_module_enable;
    storage_module_enable.val = 0;

    storage_module_enable.bit.internal = true;
    storage_module_enable.bit.external = FLASH_CHIP_STATE;

    while(1)
    {
        if(!init)
        {
            DataPipe_Init();
            // Storage.init(storage_module_enable);

            SrvComProto.init(SrvComProto_Type_MAV, NULL);
            
            TaskSample_Init(TaskSample_Period_Def);
            TaskTelemetry_Init(TaskTelemetry_Period_def);
            TaskControl_Init(TaskControl_Period_Def);
            TaskLog_Init(TaslLog_Period_Def);
            TaskNavi_Init(TaslNavi_Period_Def);
            TaskFrameCTL_Init(TaskFrameCTL_Period_Def);

            osThreadDef(SampleTask, TaskSample_Core, osPriorityRealtime, 0, 1024);
            TaskInertial_Handle = osThreadCreate(osThread(SampleTask), NULL);

            osThreadDef(ControlTask, TaskControl_Core, osPriorityAboveNormal, 0, 1024);
            TaskControl_Handle = osThreadCreate(osThread(ControlTask), NULL);

            osThreadDef(NavTask, TaskNavi_Core, osPriorityHigh, 0, 8192);
            TaskNavi_Handle = osThreadCreate(osThread(NavTask), NULL);

#if defined SD_CARD_ENABLE_STATE 
            osThreadDef(LogTask, TaskLog_Core, osPriorityAboveNormal, 0, 4096);
            TaskLog_Handle = osThreadCreate(osThread(LogTask), NULL);
#endif
            osThreadDef(TelemtryTask, TaskTelemetry_Core, osPriorityNormal, 0, 1024);
            TaskTelemetry_Handle = osThreadCreate(osThread(TelemtryTask), NULL);

            osThreadDef(FrameCTLTask, TaskFrameCTL_Core, osPriorityNormal, 0, 2048);
            TaskFrameCTL_Handle = osThreadCreate(osThread(FrameCTLTask), NULL);

            init = true;
        }

        /* run system statistic in this task */
        osDelay(10);
    }
}
