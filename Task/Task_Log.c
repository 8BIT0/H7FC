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

// redesign log data structure include log imu data structure 

typedef struct
{
    SYSTEM_RunTime max_rt_diff;     // unit: us
    SYSTEM_RunTime period;          // unit: us
    SYSTEM_RunTime start_rt;
    SYSTEM_RunTime end_rt;
    uint32_t err_interval_cnt;
    uint64_t push_cnt;
}LogSummary_TypeDef;

/* internal variable */
static const LogData_Header_TypeDef LogIMU_Header = {
    .header = LOG_HEADER,
    .type = LOG_DATATYPE_IMU,
    .size = sizeof(SrvIMU_UnionLogData_TypeDef),
};
static FATCluster_Addr LogFolder_Cluster = ROOT_CLUSTER_ADDR;
static volatile Disk_FileObj_TypeDef LogFile_Obj;
static Disk_FATFileSys_TypeDef FATFS_Obj;
static bool LogFile_Ready = false;
static bool enable_compess = true;
static SrvIMU_UnionData_TypeDef LogIMU_Data __attribute__((section(".Perph_Section")));
static uint8_t LogCache_L2_Buf[MAX_FILE_SIZE_K(10)] TCM_ATTRIBUTE;
static uint8_t LogCache_L1_Buf[MAX_FILE_SIZE_K(32)] TCM_ATTRIBUTE;
static QueueObj_TypeDef IMULog_Queue;
static QueueObj_TypeDef IMUData_Queue;
static LogData_Reg_TypeDef LogObj_Set_Reg;
static LogData_Reg_TypeDef LogObj_Enable_Reg;
static LogData_Reg_TypeDef LogObj_Logging_Reg;
static Os_IdleObj_TypeDef LogIdleObj;
static uint8_t LogQueueBuff_Trail[MAX_FILE_SIZE_K(1) / 2] = {0};

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
static void OsIdle_Callback_LogModule(uint8_t *ptr, uint16_t len);

#define HEAP_ALLOC(var,size) \
    lzo_align_t __LZO_MMODEL var [ ((size) + (sizeof(lzo_align_t) - 1)) / sizeof(lzo_align_t) ]

static HEAP_ALLOC(wrkmem, 512);

void TaskLog_Init(void)
{
    Os_Idle_DataStream_TypeDef LogData_Stream;

    uint16_t compess_in[512] = {0};
    uint16_t compess_out[512] = {0};
    uint16_t compess_tmp[512] = {0};
    uint16_t compess_tmp_len = 0;
    uint16_t compess_out_len = 0;

    if (lzo_init() != LZO_E_OK)
        enable_compess = false;

    /* test code */
    if(enable_compess)
    {
        for(uint16_t i = 0; i < 512; i++)
        {
            compess_in[i] = i;
        }
    
        if(lzo1x_1_compress(compess_in, sizeof(compess_in), compess_out, &compess_out_len, wrkmem) == LZO_E_OK)
        {
            /* check for an incompressible block */
            if (compess_out_len >= sizeof(compess_in))
                return 0;
        }

        if(lzo1x_decompress(compess_out, compess_out_len, compess_tmp, &compess_tmp_len, NULL) != LZO_E_OK)
        {
            return 0;
        }
    }

    /* data compess test */

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
            if (Queue.create_with_buf(&IMUData_Queue, "queue imu data", LogCache_L1_Buf, sizeof(LogCache_L1_Buf)) &&
                Queue.create_with_buf(&IMULog_Queue, "queue imu log", LogCache_L2_Buf, sizeof(LogCache_L2_Buf)))
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

    Os_Regist_IdleObj(&LogIdleObj, LogData_Stream, OsIdle_Callback_LogModule);
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
            state = LogData_ToFile(&IMULog_Queue, &LogObj_Logging_Reg);
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
    // DebugPin.ctl(Debug_PB4, true);

    // DebugPin.ctl(Debug_PB4, false);
}

static Disk_Write_State LogData_ToFile(QueueObj_TypeDef *queue, LogData_Reg_TypeDef *log_reg)
{
    uint16_t log_size = 0;
    uint16_t queue_size = 0;

    if ((queue == NULL) || (Queue.size(*queue) == 0))
        return Disk_Write_Error;

    log_reg->_sec.IMU_Sec = true;
    queue_size = Queue.size(*queue);
    log_reg->_sec.IMU_Sec = false;

    // while (queue_size >= sizeof(LogQueueBuff_Trail))
    // {
    //     log_reg->_sec.IMU_Sec = true;

    //     Queue.pop(queue, LogQueueBuff_Trail, sizeof(LogQueueBuff_Trail));
    //     queue_size = Queue.size(*queue);

    //     log_reg->_sec.IMU_Sec = false;
    //     if(Disk.write(&FATFS_Obj, &LogFile_Obj, LogQueueBuff_Trail, sizeof(LogQueueBuff_Trail)) == Disk_Write_Finish)
    //         return Disk_Write_Finish;
    // }
    
    log_reg->_sec.IMU_Sec = true;
    // log_size = Queue.size(*queue);

    // if(log_size)
    // {
    //     Queue.pop(queue, LogQueueBuff_Trail, log_size);
    //     log_reg->_sec.IMU_Sec = false;

    //     return Disk.write(&FATFS_Obj, &LogFile_Obj, LogQueueBuff_Trail, log_size);
    // }
    log_reg->_sec.IMU_Sec = false;

    return Disk_Write_Contiguous;
}

static void TaskLog_PipeTransFinish_Callback(DataPipeObj_TypeDef *obj)
{
    uint64_t imu_pipe_rt_diff = 0;
    static uint64_t lst_imu_pipe_rt = 0;
    static uint32_t err_time_diff = 0;
    static uint32_t imu_opy_cnt = 0;
    static uint32_t imu_log_queue_err = 0;
    SrvIMU_UnionLogData_TypeDef LogIMU_Data_tmp;

    if ((obj == NULL) || !LogFile_Ready)
        return;

    if (LogObj_Set_Reg._sec.IMU_Sec && (obj == &IMU_Log_DataPipe) && LogObj_Enable_Reg._sec.IMU_Sec)
    {
        if ((Queue.state(IMUData_Queue) == Queue_ok) ||
            (Queue.state(IMUData_Queue) == Queue_empty))
        {
            /* convert imu data to log data format */
            LogIMU_Data_tmp.data.us_diff = (((SrvIMU_UnionData_TypeDef *)(IMU_Log_DataPipe.data_addr))->data.time_stamp - lst_imu_pipe_rt) / 10;
            LogIMU_Data_tmp.data.acc_scale = ((SrvIMU_UnionData_TypeDef *)(IMU_Log_DataPipe.data_addr))->data.acc_scale;
            LogIMU_Data_tmp.data.gyr_scale = ((SrvIMU_UnionData_TypeDef *)(IMU_Log_DataPipe.data_addr))->data.gyr_scale;

            LogIMU_Data_tmp.data.cyc = ((SrvIMU_UnionData_TypeDef *)(IMU_Log_DataPipe.data_addr))->data.cycle_cnt;

            for(uint8_t axis = Axis_X; axis < Axis_Sum; axis++)
            {
                LogIMU_Data_tmp.data.flt_acc[axis] = (int16_t)(((SrvIMU_UnionData_TypeDef *)(IMU_Log_DataPipe.data_addr))->data.flt_acc[axis] * LogIMU_Data_tmp.data.acc_scale);
                LogIMU_Data_tmp.data.flt_gyr[axis] = (int16_t)(((SrvIMU_UnionData_TypeDef *)(IMU_Log_DataPipe.data_addr))->data.flt_gyr[axis] * LogIMU_Data_tmp.data.gyr_scale);

                LogIMU_Data_tmp.data.org_acc[axis] = (int16_t)(((SrvIMU_UnionData_TypeDef *)(IMU_Log_DataPipe.data_addr))->data.org_acc[axis] * LogIMU_Data_tmp.data.acc_scale);
                LogIMU_Data_tmp.data.org_gyr[axis] = (int16_t)(((SrvIMU_UnionData_TypeDef *)(IMU_Log_DataPipe.data_addr))->data.org_gyr[axis] * LogIMU_Data_tmp.data.gyr_scale);
            }

            for(uint8_t i = 0; i < sizeof(LogIMU_Data_tmp) - sizeof(LogIMU_Data_tmp.data.check_sum); i++)
            {
                LogIMU_Data_tmp.data.check_sum += LogIMU_Data_tmp.buff[i];
            }

            if((Queue.push(&IMUData_Queue, &LogIMU_Header, LOG_HEADER_SIZE) == Queue_ok) &&
               (Queue.push(&IMUData_Queue, LogIMU_Data_tmp.buff, sizeof(LogIMU_Data_tmp)) == Queue_ok))
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

        if (!LogObj_Logging_Reg._sec.IMU_Sec)
        {
            if(!Queue.pop_to_queue(&IMUData_Queue, &IMULog_Queue))
            {
                imu_log_queue_err++;
            }
        }
    }
}
