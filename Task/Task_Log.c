/*
 *  coder: 8_B!T0
 *  bref: use this task Log Gyro and Acc Data
 */
#include "Task_Log.h"
#include "scheduler.h"
#include "shell.h"
#include "debug_util.h"
#include <stdio.h>
#include "queue.h"
#include "error_log.h"
#include "mmu.h"
#include "DiskIO.h"
#include "DataPipe/DataPipe.h"
#include "Task_Protocol.h"
#include "Task_SensorInertial.h"
#include "Dev_Led.h"
#include "IO_Definition.h"
#include <stdio.h>

/* internal variable */
static FATCluster_Addr LogFolder_Cluster = ROOT_CLUSTER_ADDR;
static Disk_FileObj_TypeDef LogFile_Obj;
static Disk_FATFileSys_TypeDef FATFS_Obj;
static uint8_t test[512] = {0};
static SrvIMU_UnionData_TypeDef LogPriIMU_Data __attribute__((section(".Perph_Section")));
static SrvIMU_UnionData_TypeDef LogSecIMU_Data __attribute__((section(".Perph_Section")));
DataPipeObj_TypeDef IMU_Log_DataPipe;

/* internal function */
static void TaskLog_DataFormat_Write(const char *format, ...);

void TaskLog_Init(void)
{
    memset(&IMU_Log_DataPipe, NULL, sizeof(IMU_Log_DataPipe));
    memset(&LogFile_Obj, NULL, sizeof(LogFile_Obj));
    memset(&FATFS_Obj, NULL, sizeof(FATFS_Obj));

    IMU_Log_DataPipe.data_addr = (uint32_t)&LogPriIMU_Data;
    IMU_Log_DataPipe.data_size = sizeof(LogPriIMU_Data);

    /* init module first then init task */
    Disk.init(&FATFS_Obj, TaskProto_PushProtocolQueue);
}

Task *test_task;

void TaskLog_Core(Task_Handle hdl)
{
    static bool crt_file = false;
    static uint8_t i = 0;
    static bool led_state = false;
    static uint32_t t;

    t = Get_CurrentRunningMs();

    test_task = (Task *)hdl;

    if (!crt_file)
    {
        LogFolder_Cluster = Disk.create_folder(&FATFS_Obj, "log/", ROOT_CLUSTER_ADDR);
        LogFile_Obj = Disk.create_file(&FATFS_Obj, "log.txt", LogFolder_Cluster);
        Disk.open(&FATFS_Obj, "log/", "log.txt", &LogFile_Obj);

        crt_file = true;
    }
    else
    {
        // DebugPin.ctl(Debug_PB4, true);
        if (i < 10)
        {
            i++;
        }
        else
        {
            i = 0;
            led_state = !led_state;
            DevLED.ctl(Led2, led_state);
        }

        if (LogFile_Obj.info.size < 4 * 1024 * 1024)
        {
            TaskLog_DataFormat_Write("%ld\r\n", t);
            // Disk.write(&FATFS_Obj, &LogFile_Obj,
            // "test test test test test test test test test test test test test test test test\r\n",
            // strlen("test test test test test test test test test test test test test test test test\r\n"));
        }
        else
        {
            i = 0;
            DevLED.ctl(Led2, false);
        }

        // DebugPin.ctl(Debug_PB4, false);
    }
}

static void TaskLog_DataFormat_Write(const char *format, ...)
{
    va_list arp;

    va_start(arp, format);
    volatile uint32_t length = vsnprintf((char *)test, sizeof(test), (char *)format, arp);
    length += 1;
    Disk.write(&FATFS_Obj, &LogFile_Obj, test, length);

    va_end(arp);
}
