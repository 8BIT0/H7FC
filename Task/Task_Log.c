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

/* internal variable */
static FATCluster_Addr LogFolder_Cluster = ROOT_CLUSTER_ADDR;
static Disk_FileObj_TypeDef LogFile_Obj;
/* internal function */

void TaskLog_Init(void)
{
    memset(&LogFile_Obj, NULL, sizeof(LogFile_Obj));

    LogFolder_Cluster = Disk_Create_Folder(&FATFs_Obj, "test4/", ROOT_CLUSTER_ADDR);
    LogFile_Obj = Disk_Create_File(&FATFs_Obj, "test.txt", LogFolder_Cluster);
    Disk_Open(&FATFs_Obj, "test4/", "test.txt", &LogFile_Obj);
}

void TaskLog_Core(void)
{
}
