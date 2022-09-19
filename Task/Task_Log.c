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
#include "DataPool.h"
#include "Task_Protocol.h"
#include "Dev_Led.h"
#include "IO_Definition.h"
#include <stdio.h>

/* internal variable */
static FATCluster_Addr LogFolder_Cluster = ROOT_CLUSTER_ADDR;
static Disk_FileObj_TypeDef LogFile_Obj;
static Disk_FATFileSys_TypeDef FATFS_Obj;
static uint8_t test[512] = {0};

/* internal function */
static void TaskLog_DataFormat_Write(const char *format, ...);

void TaskLog_Init(void)
{
    memset(&LogFile_Obj, NULL, sizeof(LogFile_Obj));
    memset(&FATFS_Obj, NULL, sizeof(FATFS_Obj));

    /* init module first then init task */
    Disk.init(&FATFS_Obj, TaskProto_PushProtocolQueue);

    LogFolder_Cluster = Disk.create_folder(&FATFS_Obj, "log/", ROOT_CLUSTER_ADDR);
    LogFile_Obj = Disk.create_file(&FATFS_Obj, "log.txt", LogFolder_Cluster);
    Disk.open(&FATFS_Obj, "log/", "log.txt", &LogFile_Obj);
}

void TaskLog_Core(Task_Handle hdl)
{
    static uint8_t i = 0;
    static bool led_state = false;
    static uint32_t t;

    t = Get_CurrentRunningMs();

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

    TaskLog_DataFormat_Write("%ld\r\n", t);
    // DebugPin.ctl(Debug_PB4, false);
}

static void TaskLog_DataFormat_Write(const char *format, ...)
{
    va_list arp;

    Kernel_EnterCritical();
    va_start(arp, format);

    volatile uint32_t length = vsnprintf((char *)test, sizeof(test), (char *)format, arp);
    Disk.write(&FATFS_Obj, &LogFile_Obj, test, length);
    Kernel_ExitCritical();

    memset(test, NULL, sizeof(test));

    va_end(arp);
}
