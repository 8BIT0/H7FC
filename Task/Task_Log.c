/*
 *  coder: 8_B!T0
 *  bref: use this task Log Gyro and Acc Data
 *
 *  this task and module can be improved and optimized in a big but way we use it in short time temporary
 *  after when the main fundamental ability was accomplished we go back in this file and do improve and optimize to it
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

#define LOG_FOLDER "log/"
#define IMU_LOG_FILE "imu.log"

#define K_BYTE 1024
#define M_BYTE K_BYTE *K_BYTE
#define MAX_FILE_SIZE_M(x) x *M_BYTE
#define MAX_FILE_SIZE_K(x) x *K_BYTE
#define MIN_CACHE_NUM 2

/* internal variable */
static const LogData_Header_TypeDef LogIMU_Header = {
    .header = LOG_HEADER,
    .type = LOG_DATATYPE_IMU,
    .size = sizeof(SrvIMU_UnionData_TypeDef),
};
static FATCluster_Addr LogFolder_Cluster = ROOT_CLUSTER_ADDR;
static Disk_FileObj_TypeDef LogFile_Obj;
static Disk_FATFileSys_TypeDef FATFS_Obj;
static bool LogFile_Ready = false;
static SrvIMU_UnionData_TypeDef LogPriIMU_Data __attribute__((section(".Perph_Section")));
static SrvIMU_UnionData_TypeDef LogSecIMU_Data __attribute__((section(".Perph_Section")));
static QueueObj_TypeDef LogQueue_IMU;
static LogData_Reg_TypeDef LogObj_Set_Reg;
DataPipeObj_TypeDef IMU_Log_DataPipe;

/* internal function */
static void TaskLog_PipeTransFinish_Callback(DataPipeObj_TypeDef *obj);
static void TaskLog_ToFile(QueueObj_TypeDef *queue);

void TaskLog_Init(void)
{
    memset(&IMU_Log_DataPipe, NULL, sizeof(IMU_Log_DataPipe));
    memset(&LogFile_Obj, NULL, sizeof(LogFile_Obj));
    memset(&FATFS_Obj, NULL, sizeof(FATFS_Obj));

    IMU_Log_DataPipe.data_addr = (uint32_t)&LogPriIMU_Data;
    IMU_Log_DataPipe.data_size = sizeof(LogPriIMU_Data);
    IMU_Log_DataPipe.trans_finish_cb = TaskLog_PipeTransFinish_Callback;

    LogObj_Set_Reg.reg_val = 0;
    LogObj_State_Reg.reg_val = 0;

    /* init module first then init task */
    if (Disk.init(&FATFS_Obj, TaskProto_PushProtocolQueue))
    {
        LogFolder_Cluster = Disk.create_folder(&FATFS_Obj, LOG_FOLDER, ROOT_CLUSTER_ADDR);
        LogFile_Obj = Disk.create_file(&FATFS_Obj, IMU_LOG_FILE, LogFolder_Cluster);
        Disk.open(&FATFS_Obj, LOG_FOLDER, IMU_LOG_FILE, &LogFile_Obj);

        /* create cache queue for IMU Data */
        if (Queue.create(&LogQueue_IMU, "log queue imu", MAX_FILE_SIZE_K(10)))
        {
            LogFile_Ready = true;

            LogObj_Set_Reg._sec.IMU_Sec = true;
        }
    }
}

void TaskLog_Core(Task_Handle hdl)
{
    static uint8_t i = 0;
    static bool led_state = false;
    static uint32_t t;

    t = Get_CurrentRunningMs();

    if (LogFile_Ready)
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

        if (LogFile_Obj.info.size < MAX_FILE_SIZE_M(4))
        {
            if (LogObj_Set_Reg._sec.IMU_Sec)
            {
                TaskLog_ToFile(&LogQueue_IMU);
                LogObj_State_Reg._sec.IMU_Sec = false;
            }
        }
        else
        {
            i = 0;
            DevLED.ctl(Led2, false);
        }

        // DebugPin.ctl(Debug_PB4, false);
    }
}

static void TaskLog_ToFile(QueueObj_TypeDef *queue)
{
    uint8_t data_tmp;

    if (queue == NULL)
        return;

    for (uint16_t i = 0; i < Disk.get_min_write_unit(); i++)
    {
        if (Queue.pop(queue, &data_tmp, 1) == Queue_ok)
            Disk.write(&FATFS_Obj, &LogFile_Obj, &data_tmp, 1);
    }
}

static void TaskLog_PipeTransFinish_Callback(DataPipeObj_TypeDef *obj)
{
    if ((obj == NULL) || !LogFile_Ready)
        return;

    if (LogObj_Set_Reg._sec.IMU_Sec && (obj == &IMU_Log_DataPipe))
    {
        if ((Queue.state(LogQueue_IMU) == Queue_ok) || (Queue.state(LogQueue_IMU) == Queue_empty))
        {
            Queue.push(&LogQueue_IMU, &LogIMU_Header, LOG_HEADER_SIZE);
            Queue.push(&LogQueue_IMU, (uint8_t *)(IMU_Log_DataPipe.data_addr), IMU_Log_DataPipe.data_size);
        }
    }
}
