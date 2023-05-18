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
#include "minilzo.h"
#include <stdio.h>

#define LOG_FOLDER "log/"
#define IMU_LOG_FILE "imu.log"

#define K_BYTE 1024
#define M_BYTE (K_BYTE * K_BYTE)
#define MAX_FILE_SIZE_M(x) (x * M_BYTE)
#define MAX_FILE_SIZE_K(x) (x * K_BYTE)
#define MIN_CACHE_NUM 2

#define LOG_COMPESS_HEADER 0xCA
#define LOG_COMPESS_ENDER 0xED

#define HEAP_ALLOC(var,size) \
    lzo_align_t __LZO_MMODEL var [ ((size) + (sizeof(lzo_align_t) - 1)) / sizeof(lzo_align_t) ] TCM_ATTRIBUTE

static HEAP_ALLOC(wrkmem, LZO1X_1_MEM_COMPRESS);

typedef struct
{
    SYSTEM_RunTime max_rt_diff;     // unit: us
    SYSTEM_RunTime period;          // unit: us
    SYSTEM_RunTime start_rt;
    SYSTEM_RunTime end_rt;
    uint32_t err_interval_cnt;
    uint64_t push_cnt;
}LogSummary_TypeDef;

#pragma pack(1)
typedef struct
{
    const uint8_t mark;
    uint16_t size;
    uint8_t cmps_buf[MAX_FILE_SIZE_K(1)];
}LogCompess_StreamTypeDef;

typedef union
{
    LogCompess_StreamTypeDef stream;
    uint8_t buff[sizeof(LogCompess_StreamTypeDef)];
}LogCompess_UnionTypeDef;
#pragma pack()

/* internal variable */
static const LogData_Header_TypeDef LogIMU_Header = {
    .header = LOG_HEADER,
    .type = LOG_DATATYPE_IMU,
    .size = sizeof(LogIMUDataUnion_TypeDef),
};
static LogCompess_UnionTypeDef LogCompess_Stream TCM_ATTRIBUTE = {
    .stream = {
        .mark = LOG_COMPESS_HEADER,
        .size = 0,
    },
};
static FATCluster_Addr LogFolder_Cluster = ROOT_CLUSTER_ADDR;
static volatile Disk_FileObj_TypeDef LogFile_Obj;
static Disk_FATFileSys_TypeDef FATFS_Obj;
static bool LogFile_Ready = false;
static bool enable_compess = true;
static SrvIMU_UnionData_TypeDef LogIMU_Data __attribute__((section(".Perph_Section")));
static uint8_t LogCache_L1_Buf[MAX_FILE_SIZE_K(4)] TCM_ATTRIBUTE;
static uint8_t LogCache_L2_Buf[MAX_FILE_SIZE_K(4)] TCM_ATTRIBUTE;
static QueueObj_TypeDef IMUData_Queue;
static LogData_Reg_TypeDef LogObj_Set_Reg;
static LogData_Reg_TypeDef LogObj_Enable_Reg;
static LogData_Reg_TypeDef LogObj_Logging_Reg;
static Os_IdleObj_TypeDef LogIdleObj;
static uint8_t LogQueueBuff_Trail[MAX_FILE_SIZE_K(1) / 2] = {0};
static uint16_t QueueIMU_PopSize = 0;


static LogSummary_TypeDef LogIMU_Summary = {
    .max_rt_diff = 0,
    .period = 500,
    .err_interval_cnt = 0,
    .push_cnt = 0,
    .start_rt = 0,
    .end_rt = 0,
};

/* internal function */
static void TaskLog_PipeTransFinish_Callback(DataPipeObj_TypeDef *obj);
static Disk_Write_State LogData_ToFile(QueueObj_TypeDef *queue, LogData_Reg_TypeDef *log_reg);

void TaskLog_Init(void)
{
    Os_Idle_DataStream_TypeDef LogData_Stream;

    if (lzo_init() != LZO_E_OK)
        enable_compess = false;

    memset(&IMU_Log_DataPipe, NULL, sizeof(IMU_Log_DataPipe));
    memset(&LogFile_Obj, NULL, sizeof(LogFile_Obj));
    memset(&FATFS_Obj, NULL, sizeof(FATFS_Obj));

    IMU_Log_DataPipe.data_addr = (uint32_t)&LogIMU_Data;
    IMU_Log_DataPipe.data_size = sizeof(LogIMU_Data);
    IMU_Log_DataPipe.trans_finish_cb = TaskLog_PipeTransFinish_Callback;

    LogObj_Set_Reg.reg_val = 0;
    LogObj_Logging_Reg.reg_val = 0;

    /* init module first then init task */
    if (Disk.init(&FATFS_Obj, TaskProto_PushProtocolQueue))
    {
        LogFolder_Cluster = Disk.create_folder(&FATFS_Obj, LOG_FOLDER, ROOT_CLUSTER_ADDR);

        if(LogFolder_Cluster)
        {
            LogFile_Obj = Disk.create_file(&FATFS_Obj, IMU_LOG_FILE, LogFolder_Cluster, MAX_FILE_SIZE_M(2));
            Disk.open(&FATFS_Obj, LOG_FOLDER, IMU_LOG_FILE, &LogFile_Obj);

            /* create cache queue for IMU Data */
            if (Queue.create_with_buf(&IMUData_Queue, "queue imu data", LogCache_L1_Buf, sizeof(LogCache_L1_Buf)))
            {
                LogFile_Ready = true;
                LogObj_Enable_Reg._sec.IMU_Sec = true;
                LogObj_Set_Reg._sec.IMU_Sec = true;
            }

            DataPipe_Enable(&IMU_Log_DataPipe);
        }
        else
        {
            LogFile_Ready = false;
            LogObj_Enable_Reg._sec.IMU_Sec = false;
            LogObj_Set_Reg._sec.IMU_Sec = false;
            DataPipe_Disable(&IMU_Log_DataPipe);
        }
    }
    else
        DataPipe_Disable(&IMU_Log_DataPipe);

    LogData_Stream.ptr = NULL;
    LogData_Stream.size = 0;
}

static void OsIdle_Callback_LogModule(uint8_t *ptr, uint16_t len)
{
    Disk_Write_State state;
    static uint32_t rt = 0;
    static uint32_t rt_lst = 0;
    static bool led_state = false;

    if (LogFile_Ready)
    {
        if (LogObj_Set_Reg._sec.IMU_Sec)
        {
            // state = LogData_ToFile(&IMULog_Queue, &LogObj_Logging_Reg);
            rt = Get_CurrentRunningMs();

            if (state == Disk_Write_Contiguous)
            {
                if (rt - rt_lst >= 200)
                {
                    led_state = !led_state;
                    DevLED.ctl(Led1, led_state);
                    rt_lst = rt;
                }
            }
        }
        
        if (state == Disk_Write_Finish)
        {
            LogObj_Set_Reg._sec.IMU_Sec = false;
            DataPipe_Disable(&IMU_Log_DataPipe);

            DevLED.ctl(Led1, false);
        }
        else
        {
            if (rt - rt_lst >= 500)
            {
                led_state = !led_state;
                DevLED.ctl(Led1, led_state);
                rt_lst = rt;
            }
        }
    }
}

void TaskLog_Core(Task_Handle hdl)
{
    static bool compess = false;

    // DebugPin.ctl(Debug_PB4, true);
    LogObj_Logging_Reg._sec.IMU_Sec = true;
    if(enable_compess && QueueIMU_PopSize)
    {
        if(!compess)
        {
            if(lzo1x_1_compress(LogCache_L2_Buf, QueueIMU_PopSize, LogCompess_Stream.stream.cmps_buf, &LogCompess_Stream.stream.size, wrkmem) != LZO_E_OK)
            {
                enable_compess = false;
                DataPipe_Disable(&IMU_Log_DataPipe);
            }
            else
                compess = true;
        }
        
        if(QueueIMU_PopSize >= 512)
        {
            QueueIMU_PopSize -= 512;
        }
        else
        {
            QueueIMU_PopSize = 0;
            compess = false;
        }

        if(QueueIMU_PopSize == 0)
            LogObj_Logging_Reg._sec.IMU_Sec = false;
    }
    // DebugPin.ctl(Debug_PB4, false);
}

static Disk_Write_State LogData_ToFile(QueueObj_TypeDef *queue, LogData_Reg_TypeDef *log_reg)
{
    uint16_t log_size = 0;
    uint16_t queue_size = 0;

    if ((queue == NULL) || (Queue.size(*queue) == 0))
        return Disk_Write_Error;

    // while (queue_size >= sizeof(LogQueueBuff_Trail))
    // {
    //     log_reg->_sec.IMU_Sec = true;

    //     Queue.pop(queue, LogQueueBuff_Trail, sizeof(LogQueueBuff_Trail));
    //     queue_size = Queue.size(*queue);

    //     log_reg->_sec.IMU_Sec = false;
    //     if(Disk.write(&FATFS_Obj, &LogFile_Obj, LogQueueBuff_Trail, sizeof(LogQueueBuff_Trail)) == Disk_Write_Finish)
    //         return Disk_Write_Finish;
    // }
    
    
    // log_size = Queue.size(*queue);

    // if(log_size)
    // {
    //     Queue.pop(queue, LogQueueBuff_Trail, log_size);
    //     log_reg->_sec.IMU_Sec = false;

    //     return Disk.write(&FATFS_Obj, &LogFile_Obj, LogQueueBuff_Trail, log_size);
    // }

    return Disk_Write_Contiguous;
}

static void TaskLog_PipeTransFinish_Callback(DataPipeObj_TypeDef *obj)
{
    uint64_t imu_pipe_rt_diff = 0;
    static uint64_t lst_imu_pipe_rt = 0;
    static uint32_t imu_opy_cnt = 0;
    LogIMUDataUnion_TypeDef Log_Buf;

    if ((obj == NULL) || !LogFile_Ready)
        return;

    if (LogObj_Set_Reg._sec.IMU_Sec && (obj == &IMU_Log_DataPipe) && LogObj_Enable_Reg._sec.IMU_Sec)
    {
        Log_Buf.data.time = ((SrvIMU_UnionData_TypeDef *)(IMU_Log_DataPipe.data_addr))->data.time_stamp;
        Log_Buf.data.acc_scale = ((SrvIMU_UnionData_TypeDef *)(IMU_Log_DataPipe.data_addr))->data.acc_scale;
        Log_Buf.data.gyr_scale = ((SrvIMU_UnionData_TypeDef *)(IMU_Log_DataPipe.data_addr))->data.gyr_scale;
        Log_Buf.data.cyc = ((SrvIMU_UnionData_TypeDef *)(IMU_Log_DataPipe.data_addr))->data.cycle_cnt & 0x000000FF;
        
        for(uint8_t axis = Axis_X; axis < Axis_Sum; axis ++)
        {
            Log_Buf.data.flt_acc[axis] = (int16_t)(Log_Buf.data.acc_scale * ((SrvIMU_UnionData_TypeDef *)(IMU_Log_DataPipe.data_addr))->data.flt_acc[axis]);
            Log_Buf.data.flt_gyr[axis] = (int16_t)(Log_Buf.data.gyr_scale * ((SrvIMU_UnionData_TypeDef *)(IMU_Log_DataPipe.data_addr))->data.flt_gyr[axis]);
        
            Log_Buf.data.org_acc[axis] = (int16_t)(Log_Buf.data.acc_scale * ((SrvIMU_UnionData_TypeDef *)(IMU_Log_DataPipe.data_addr))->data.org_acc[axis]);
            Log_Buf.data.org_gyr[axis] = (int16_t)(Log_Buf.data.gyr_scale * ((SrvIMU_UnionData_TypeDef *)(IMU_Log_DataPipe.data_addr))->data.org_gyr[axis]);
        }

        for(uint8_t i = 0; i < sizeof(LogIMUDataUnion_TypeDef) - sizeof(uint8_t); i++)
        {
            Log_Buf.data.check_sum += Log_Buf.buff[i];
        }

        if ((Queue.state(IMUData_Queue) == Queue_ok) ||
            (Queue.state(IMUData_Queue) == Queue_empty))
        {
            if((Queue.push(&IMUData_Queue, &LogIMU_Header, LOG_HEADER_SIZE) == Queue_ok) &&
               (Queue.push(&IMUData_Queue, Log_Buf.buff, sizeof(Log_Buf)) == Queue_ok))
            {
                if(LogIMU_Summary.start_rt == 0)
                    LogIMU_Summary.start_rt = ((SrvIMU_UnionData_TypeDef *)(IMU_Log_DataPipe.data_addr))->data.time_stamp;

                LogIMU_Summary.push_cnt ++;

                imu_pipe_rt_diff = ((SrvIMU_UnionData_TypeDef *)(IMU_Log_DataPipe.data_addr))->data.time_stamp - lst_imu_pipe_rt;

                if(imu_pipe_rt_diff > LogIMU_Summary.period)
                {
                    if(imu_pipe_rt_diff > LogIMU_Summary.max_rt_diff)
                        LogIMU_Summary.max_rt_diff = imu_pipe_rt_diff;

                    LogIMU_Summary.err_interval_cnt ++;
                }

                LogIMU_Summary.end_rt = ((SrvIMU_UnionData_TypeDef *)(IMU_Log_DataPipe.data_addr))->data.time_stamp;
                lst_imu_pipe_rt = ((SrvIMU_UnionData_TypeDef *)(IMU_Log_DataPipe.data_addr))->data.time_stamp;
            }
        }
        else
        {
            imu_opy_cnt++;
        }

        if(!LogObj_Logging_Reg._sec.IMU_Sec && 
            enable_compess && 
            Queue.size(IMUData_Queue) >= MAX_FILE_SIZE_K(1))
        {
            QueueIMU_PopSize = (MAX_FILE_SIZE_K(1) / (LOG_HEADER_SIZE + sizeof(Log_Buf))) * (LOG_HEADER_SIZE + sizeof(Log_Buf));
            if(QueueIMU_PopSize <= sizeof(LogCache_L2_Buf))
            {
                Queue.pop(&IMUData_Queue, LogCache_L2_Buf, QueueIMU_PopSize);
            }
        }
    }
}
