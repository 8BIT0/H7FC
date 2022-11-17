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
static volatile Disk_FileObj_TypeDef LogFile_Obj;
static Disk_FATFileSys_TypeDef FATFS_Obj;
static bool LogFile_Ready = false;
static SrvIMU_UnionData_TypeDef LogPriIMU_Data __attribute__((section(".Perph_Section")));
static SrvIMU_UnionData_TypeDef LogSecIMU_Data __attribute__((section(".Perph_Section")));
static uint8_t LogCache_L2_Buf[MAX_FILE_SIZE_K(10)] __attribute__((section(".Fast_Mem_section")));
static QueueObj_TypeDef IMULog_Queue;
static QueueObj_TypeDef IMUData_Queue;
static LogData_Reg_TypeDef LogObj_Set_Reg;
static LogData_Reg_TypeDef LogObj_Enable_Reg;
static LogData_Reg_TypeDef LogObj_Logging_Reg;
DataPipeObj_TypeDef IMU_Log_DataPipe;
static Os_IdleObj_TypeDef LogIdleObj;
static uint8_t LogQueueBuff_Trail[MAX_FILE_SIZE_K(1) / 2] = {0};

/* internal function */
static void TaskLog_PipeTransFinish_Callback(DataPipeObj_TypeDef *obj);
static bool LogData_ToFile(QueueObj_TypeDef *queue, DataPipeObj_TypeDef pipe_obj, LogData_Reg_TypeDef *log_reg);
static void OsIdle_Callback_LogModule(uint8_t *ptr, uint16_t len);

void TaskLog_Init(void)
{
    Os_Idle_DataStream_TypeDef LogData_Stream;

    memset(&IMU_Log_DataPipe, NULL, sizeof(IMU_Log_DataPipe));
    memset(&LogFile_Obj, NULL, sizeof(LogFile_Obj));
    memset(&FATFS_Obj, NULL, sizeof(FATFS_Obj));

    IMU_Log_DataPipe.data_addr = (uint32_t)&LogPriIMU_Data;
    IMU_Log_DataPipe.data_size = sizeof(LogPriIMU_Data);
    IMU_Log_DataPipe.trans_finish_cb = TaskLog_PipeTransFinish_Callback;
    IMU_Log_DataPipe.ptr_tmp = MMU_Malloc(IMU_Log_DataPipe.data_addr);

    LogObj_Set_Reg.reg_val = 0;
    LogObj_Logging_Reg.reg_val = 0;

    /* init module first then init task */
    if (Disk.init(&FATFS_Obj, TaskProto_PushProtocolQueue))
    {
        LogFolder_Cluster = Disk.create_folder(&FATFS_Obj, LOG_FOLDER, ROOT_CLUSTER_ADDR);
        LogFile_Obj = Disk.create_file(&FATFS_Obj, IMU_LOG_FILE, LogFolder_Cluster, MAX_FILE_SIZE_M(16));
        Disk.open(&FATFS_Obj, LOG_FOLDER, IMU_LOG_FILE, &LogFile_Obj);

        /* create cache queue for IMU Data */
        if (Queue.create_auto(&IMUData_Queue, "queue imu data", MAX_FILE_SIZE_K(10)) &&
            Queue.create_with_buf(&IMULog_Queue, "queue imu log", LogCache_L2_Buf, sizeof(LogCache_L2_Buf)))
        {
            LogFile_Ready = true;
            LogObj_Enable_Reg._sec.IMU_Sec = true;
            LogObj_Set_Reg._sec.IMU_Sec = true;
        }
    }

    LogData_Stream.ptr = NULL;
    LogData_Stream.size = 0;

    Os_Regist_IdleObj(&LogIdleObj, LogData_Stream, OsIdle_Callback_LogModule);
}

static void OsIdle_Callback_LogModule(uint8_t *ptr, uint16_t len)
{
    static uint32_t rt = 0;
    static uint32_t rt_lst = 0;
    static uint32_t lst_size = 0;
    static bool led_state = false;

    if (LogFile_Ready)
    {
        if (LogObj_Set_Reg._sec.IMU_Sec)
        {
            if (LogFile_Obj.info.size < MAX_FILE_SIZE_M(16))
            {
                LogData_ToFile(&IMULog_Queue, IMU_Log_DataPipe, &LogObj_Logging_Reg);
                rt = Get_CurrentRunningMs();

                if(LogFile_Obj.info.size != lst_size)
                {
                    if(rt - rt_lst >= 200)
                    {
                        led_state = !led_state;
                        DevLED.ctl(Led2, led_state);
                        rt_lst = rt;
                    }
                }
                else
                    DevLED.ctl(Led2, true);

                lst_size = LogFile_Obj.info.size;
            }
            else
            {
                LogObj_Enable_Reg._sec.IMU_Sec = false;
                DevLED.ctl(Led2, false);
            }
        }
    }
}

void TaskLog_Core(Task_Handle hdl)
{
    // DebugPin.ctl(Debug_PB4, true);

    // DebugPin.ctl(Debug_PB4, false);
}

static bool LogData_ToFile(QueueObj_TypeDef *queue, DataPipeObj_TypeDef pipe_obj, LogData_Reg_TypeDef *log_reg)
{
    uint16_t log_size = 0;
    uint16_t queue_size = 0;

    if ((queue == NULL) || (Queue.size(*queue) == 0) || (pipe_obj.ptr_tmp == NULL))
        return;

    log_reg->_sec.IMU_Sec = true;
    queue_size = Queue.size(*queue);

    if (queue_size > sizeof(LogQueueBuff_Trail))
    {
        log_size = sizeof(LogQueueBuff_Trail);
    }
    else
        log_size = queue_size;

    if (log_size)
        Queue.pop(queue, LogQueueBuff_Trail, log_size);

    log_reg->_sec.IMU_Sec = false;

    if (log_size)
        return Disk.write(&FATFS_Obj, &LogFile_Obj, LogQueueBuff_Trail, log_size);

    __DSB();

    return false;
}

static void TaskLog_PipeTransFinish_Callback(DataPipeObj_TypeDef *obj)
{
    if ((obj == NULL) || !LogFile_Ready)
        return;

    if (LogObj_Set_Reg._sec.IMU_Sec && (obj == &IMU_Log_DataPipe) && LogObj_Enable_Reg._sec.IMU_Sec)
    {
        if ((Queue.state(IMUData_Queue) == Queue_ok) ||
            (Queue.state(IMUData_Queue) == Queue_empty))
        {
            Queue.push(&IMUData_Queue, &LogIMU_Header, LOG_HEADER_SIZE);
            Queue.push(&IMUData_Queue, (uint8_t *)(IMU_Log_DataPipe.data_addr), IMU_Log_DataPipe.data_size);
        }

        if (!LogObj_Logging_Reg._sec.IMU_Sec)
        {
            Queue.pop_to_queue(&IMUData_Queue, &IMULog_Queue);
        }
    }
}
