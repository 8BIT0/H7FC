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
#define M_BYTE K_BYTE * K_BYTE
#define MAX_FILE_SIZE_M(x) x * M_BYTE
#define MAX_FILE_SIZE_K(x) x * K_BYTE
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
DataPipeObj_TypeDef IMU_Log_DataPipe;
Log_Monitor_TypeDef IMU_LogMonitor;

/* internal function */
static void TaskLog_PipeTransFinish_Callback(DataPipeObj_TypeDef *obj);
static bool TaskLog_CreateCache(Log_Monitor_TypeDef *obj, uint8_t cache_sum, uint16_t cache_buff_size, LogData_Header_TypeDef header);
static void TaskLog_IMU_ToFile(Log_Monitor_TypeDef *log_obj);

void TaskLog_Init(void)
{
    memset(&IMU_Log_DataPipe, NULL, sizeof(IMU_Log_DataPipe));
    memset(&LogFile_Obj, NULL, sizeof(LogFile_Obj));
    memset(&FATFS_Obj, NULL, sizeof(FATFS_Obj));

    IMU_Log_DataPipe.data_addr = (uint32_t)&LogPriIMU_Data;
    IMU_Log_DataPipe.data_size = sizeof(LogPriIMU_Data);
    IMU_Log_DataPipe.trans_finish_cb = TaskLog_PipeTransFinish_Callback;

    /* init module first then init task */
    if (Disk.init(&FATFS_Obj, TaskProto_PushProtocolQueue))
    {
        LogFolder_Cluster = Disk.create_folder(&FATFS_Obj, LOG_FOLDER, ROOT_CLUSTER_ADDR);
        LogFile_Obj = Disk.create_file(&FATFS_Obj, IMU_LOG_FILE, LogFolder_Cluster);
        Disk.open(&FATFS_Obj, LOG_FOLDER, IMU_LOG_FILE, &LogFile_Obj);

        /* create cache 3K for IMU Data */
        if (TaskLog_CreateCache(&IMU_LogMonitor, 6, FATFS_Obj.BytePerSection, LogIMU_Header))
            LogFile_Ready = true;
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
            TaskLog_IMU_ToFile(&IMU_LogMonitor);
        }
        else
        {
            i = 0;
            DevLED.ctl(Led2, false);
        }

        // DebugPin.ctl(Debug_PB4, false);
    }
}

static bool TaskLog_CreateCache(Log_Monitor_TypeDef *obj, uint8_t cache_sum, uint16_t cache_buff_size, LogData_Header_TypeDef header)
{
    uint16_t init_cnt = 0;

    if ((obj == NULL) || (cache_sum <= 1) || (cache_buff_size == 0))
        return false;

    obj->cache_obj = (struct LogCache_TypeDef *)MMU_Malloc(sizeof(struct LogCache_TypeDef) * cache_sum);
    if (obj->cache_obj == NULL)
    {
        MMU_Free(obj->cache_obj);
        return false;
    }

    memcpy(&obj->log_header, &header, sizeof(header));
    obj->single_log_size = sizeof(LogData_Header_TypeDef) + header.size;
    obj->single_log_offset = sizeof(LogData_Header_TypeDef);

    init_cnt = cache_buff_size / obj->single_log_size;

    for (uint8_t i = 0; i < cache_sum; i++)
    {
        memset(&(obj->cache_obj)[i], NULL, sizeof(struct LogCache_TypeDef));

        obj->cache_obj[i].p_buf = MMU_Malloc(cache_buff_size);
        if (obj->cache_obj[i].p_buf == NULL)
        {
            MMU_Free(obj->cache_obj[i].p_buf);
            MMU_Free(obj->cache_obj);
            return false;
        }

        memset(obj->cache_obj[i].p_buf, NULL, cache_buff_size);
        obj->cache_obj[i].nxt = NULL;
        obj->cache_obj[i].rem_size = cache_buff_size;
        obj->cache_obj[i].tot_size = cache_buff_size;
        obj->cache_obj[i].ocp_size = 0;

        for (uint8_t init_index = 0; init_index < init_cnt; init_index++)
        {
            memcpy(&obj->cache_obj[i].p_buf[init_index * obj->single_log_size], &header, sizeof(header));
        }

        if(i > 0)
        {
            obj->cache_obj[i - 1].nxt = &obj->cache_obj[i];
        }
    }

    obj->inuse_cache_page = obj->cache_obj;
    obj->store_page = NULL;

    return true;
}

static void TaskLog_IMU_ToFile(Log_Monitor_TypeDef *log_obj)
{
    if (log_obj == NULL || log_obj->store_page == NULL)
        return;

    Disk.write(&FATFS_Obj, &LogFile_Obj, log_obj->store_page->p_buf, log_obj->store_page->ocp_size);

    memset(log_obj->store_page->p_buf, NULL, log_obj->store_page->tot_size);
    for (uint8_t init_index = 0; init_index < log_obj->store_page->tot_size / log_obj->single_log_size; init_index++)
    {
        memcpy(&log_obj->store_page->p_buf[init_index * log_obj->single_log_size], &log_obj->log_header, sizeof(log_obj->log_header));
    }

    log_obj->store_page->ocp_size = 0;
    log_obj->store_page->rem_size = log_obj->store_page->tot_size;

    if(log_obj->store_page->nxt == NULL)
    {
        log_obj->store_page = NULL;
    }
    else
        log_obj->store_page = log_obj->store_page->nxt;
}

static void TaskLog_IMUData_Update(Log_Monitor_TypeDef *log_obj, DataPipeObj_TypeDef *pipe_obj)
{
    if ((log_obj == NULL) || (log_obj->inuse_cache_page == NULL))
        return;

    if (log_obj->inuse_cache_page->rem_size < log_obj->single_log_size)
    {
        if(log_obj->store_page == NULL)
        {
            log_obj->store_page = log_obj->inuse_cache_page;
        }
        else
            /* if stil logging then insert to next */
            log_obj->store_page->nxt = log_obj->inuse_cache_page;

        /* switch inuse cache page */
        if(log_obj->inuse_cache_page->nxt)
        {
            log_obj->inuse_cache_page = log_obj->inuse_cache_page->nxt;
        }
        else
            log_obj->inuse_cache_page = log_obj->cache_obj;
    }
     
    if(log_obj->inuse_cache_page->rem_size - log_obj->single_log_size >= 0)
    {
        memcpy(log_obj->inuse_cache_page->p_buf + log_obj->single_log_offset + log_obj->inuse_cache_page->ocp_size,
            (uint8_t *)(pipe_obj->data_addr),
            pipe_obj->data_size);

        log_obj->inuse_cache_page->rem_size -= log_obj->single_log_size;
        log_obj->inuse_cache_page->ocp_size += log_obj->single_log_size;
    }
}

static void TaskLog_PipeTransFinish_Callback(DataPipeObj_TypeDef *obj)
{
    if (obj == NULL)
        return;

    if (obj == &IMU_Log_DataPipe)
    {
        TaskLog_IMUData_Update(&IMU_LogMonitor, &IMU_Log_DataPipe);
    }
}
