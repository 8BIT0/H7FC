#include "Task_Manager.h"
#include "Task_Log.h"
#include "Task_Control.h"
#include "Task_Sample.h"
#include "Task_Telemetry.h"
#include "Task_Navi.h"
#include "Task_Protocol.h"
#include "Task_BlackBox.h"
#include "debug_util.h"
#include "HW_Def.h"
#include "Dev_Led.h"
#include "DiskIO.h"
#include "Srv_ComProto.h"
#include "Srv_OsCommon.h"
#include "../DataPipe/DataPipe.h"
#include "shell_port.h"
#include "Storage.h"
#include "cmsis_os.h"

#define TaskSample_Period_Def    1  /* unit: ms period 1ms  1000Hz */
#define TaskControl_Period_Def   2  /* unit: ms period 2ms  500Hz  */
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
    DevLED.init(Led1);
#if defined MATEKH743_V1_5
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
#elif defined BATEAT32F435_AIO
    DebugPin.init(Debug_PC8);
    DebugPin.init(Debug_PC0);
#endif
    /* vol ADC init */

    /* cur ADC init */

    DebugPort.free = SrvOsCommon.free;
    DebugPort.malloc = SrvOsCommon.malloc;
    Debug_Port_Init(&DebugPort);

    osThreadDef(ManagerTask, Task_Manager_CreateTask, osPriorityLow, 0, 1024);
    TaskManager_Handle = osThreadCreate(osThread(ManagerTask), NULL);

    osKernelStart();
}

void Task_Manager_CreateTask(void const *arg)
{
    bool init = false;
    Storage_ExtFLashDevObj_TypeDef *storage_ExtFlashObj = NULL;

#if (FLASH_CHIP_STATE == ON)
    storage_ExtFlashObj = (Storage_ExtFLashDevObj_TypeDef *)SrvOsCommon.malloc(sizeof(Storage_ExtFLashDevObj_TypeDef));

    if (storage_ExtFlashObj)
    {
        storage_ExtFlashObj->bus_type = ExtFlash_Bus_Type;
        storage_ExtFlashObj->chip_type = ExtFlash_Chip_Type;
        storage_ExtFlashObj->api = ExtFlash_Dev_Api;
        storage_ExtFlashObj->obj = NULL;
    }
    else
    {
        SrvOsCommon.free(storage_ExtFlashObj);
        storage_ExtFlashObj = NULL;
    }
#endif

    while(1)
    {
        if (!init)
        {
            uint32_t sys_time = SrvOsCommon.get_os_ms();
            DEBUG_INFO("Sys Start\r\n");
            DEBUG_INFO("Sys Time: %d\r\n", sys_time);

            DataPipe_Init();

            Storage.init(storage_ExtFlashObj);
            SrvUpgrade.init();
            SrvComProto.init(SrvComProto_Type_MAV, NULL);
            
            TaskSample_Init(TaskSample_Period_Def);
            TaskTelemetry_Init(TaskTelemetry_Period_def);
            TaskControl_Init(TaskControl_Period_Def);
#if (SD_CARD_ENABLE_STATE == ON)
            TaskLog_Init(TaslLog_Period_Def);
#else
            TaskBlackBox_Init();
#endif
            TaskNavi_Init(TaslNavi_Period_Def);
            TaskFrameCTL_Init(TaskFrameCTL_Period_Def);

            osThreadDef(SampleTask, TaskSample_Core, osPriorityRealtime, 0, 1024);
            TaskInertial_Handle = osThreadCreate(osThread(SampleTask), NULL);

            osThreadDef(TelemtryTask, TaskTelemetry_Core, osPriorityHigh, 0, 1024);
            TaskTelemetry_Handle = osThreadCreate(osThread(TelemtryTask), NULL);

            osThreadDef(ControlTask, TaskControl_Core, osPriorityHigh, 0, 1024);
            TaskControl_Handle = osThreadCreate(osThread(ControlTask), NULL);

            osThreadDef(NavTask, TaskNavi_Core, osPriorityAboveNormal, 0, 1024);
            TaskNavi_Handle = osThreadCreate(osThread(NavTask), NULL);

#if (SD_CARD_ENABLE_STATE  == ON)
            osThreadDef(LogTask, TaskLog_Core, osPriorityAboveNormal, 0, 4096);
            TaskLog_Handle = osThreadCreate(osThread(LogTask), NULL);
#else
            osThreadDef(BlackBoxTask, TaskBlackBox_Core, osPriorityNormal, 0, 4096);
            TaskLog_Handle = osThreadCreate(osThread(BlackBoxTask), NULL);
#endif

            osThreadDef(FrameCTLTask, TaskFrameCTL_Core, osPriorityNormal, 0, 1024);
            TaskFrameCTL_Handle = osThreadCreate(osThread(FrameCTLTask), NULL);

            init = true;
        }

        /* run system statistic in this task */
        osDelay(10);
    }
}
