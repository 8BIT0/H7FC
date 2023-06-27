#include "Task_Manager.h"
#include "Task_Log.h"
#include "Task_Protocol.h"
#include "Task_Control.h"
#include "Task_SensorInertial.h"
#include "Task_Telemetry.h"
#include "debug_util.h"
#include "IO_Definition.h"
#include "Dev_Led.h"
#include "DiskIO.h"
#include "../DataPipe/DataPipe.h"
#include "cmsis_os.h"

osThreadId TaskProtocol_Handle = NULL;
osThreadId TaskInertial_Handle = NULL;
osThreadId TaskControl_Handle = NULL;
osThreadId TaskNavi_Handle = NULL;
osThreadId TaskLog_Handle = NULL;
osThreadId TaskTelemetry_Handle = NULL;


void test_PC0_ctl(void)
{
    DebugPin.ctl(Debug_PC0, true);
    DebugPin.ctl(Debug_PC0, false);
}

void test_PC1_ctl(void)
{
    DebugPin.ctl(Debug_PC1, true);
    DebugPin.ctl(Debug_PC1, false);
}

void test_PC2_ctl(void)
{
    DebugPin.ctl(Debug_PC2, true);
    DebugPin.ctl(Debug_PC2, false);
}

void Task_Manager_Init(void)
{
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

    /* vol ADC init */

    /* cur ADC init */

    DataPipe_Init();

    TaskProtocol_Init();
    TaskInertial_Init();
    TaskTelemetry_Init();
    TaskControl_Init();
    TaskLog_Init();
}

void Task_Manager_CreateTask(void)
{
    osThreadDef(SampleTask, TaskInertical_Core, osPriorityRealtime, 0, 1024);
    TaskInertial_Handle = osThreadCreate(osThread(SampleTask), NULL);

    osThreadDef(ControlTask, TaskControl_Core, osPriorityHigh, 0, 1024);
    TaskControl_Handle = osThreadCreate(osThread(ControlTask), NULL);

    // osThreadDef(NavTask, , osPriorityHigh, 0, 1024);
    // TaskNavi_Handle = osThreadCreate(osThread(NavTask), NULL);

    osThreadDef(ProtocolTask, TaskProtocol_Core, osPriorityNormal, 0, 1024);
    TaskProtocol_Handle = osThreadCreate(osThread(ProtocolTask), NULL);

    osThreadDef(LogTask, TaskLog_Core, osPriorityAboveNormal, 0, 1024);
    TaskLog_Handle = osThreadCreate(osThread(LogTask), NULL);

    osThreadDef(TelemtryTask, TaskTelemetry_Core, osPriorityNormal, 0, 512);
    TaskTelemetry_Handle = osThreadCreate(osThread(LogTask), NULL);

    osKernelStart();
}
