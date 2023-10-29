/*
 *  coder: 8_B!T0
 *  bref: use this task Log Gyro and Acc Data
 *
 *  this task and module can be improved and optimized in a big but way we use it in short time temporary
 *  after when the main fundamental ability was accomplished we go back in this file and do improve and optimize to it
 */
#include "cmsis_os.h"
#include "Task_Log.h"
#include "shell.h"
#include "debug_util.h"
#include <stdio.h>
#include "../DataStructure/CusQueue.h"
#include "error_log.h"
#include "DiskIO.h"
#include "../DataPipe/DataPipe.h"
#include "Task_Protocol.h"
#include "Task_Sample.h"
#include "Dev_Led.h"
#include "IO_Definition.h"
#include "minilzo.h"
#include "Srv_OsCommon.h"
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
    lzo_align_t __LZO_MMODEL var [ ((size) + (sizeof(lzo_align_t) - 1)) / sizeof(lzo_align_t) ]

static HEAP_ALLOC(wrkmem, LZO1X_1_MEM_COMPRESS);

typedef struct
{
    uint32_t max_rt_diff;     // unit: us
    uint32_t period;          // unit: us
    uint32_t start_rt;
    uint32_t end_rt;
    uint32_t err_interval_cnt;
    uint64_t push_cnt;
}LogSummary_TypeDef;

/* internal variable */
static const LogData_Header_TypeDef LogIMU_Header = {
    .header = LOG_HEADER,
    .type = LOG_DATATYPE_IMU,
    .size = sizeof(LogIMUDataUnion_TypeDef),
};

typedef struct
{
    uint8_t buf[MAX_FILE_SIZE_K(2)];
    uint16_t compess_size;
    uint8_t total;
}LogCompess_Data_TypeDef;

static LogCompess_Data_TypeDef LogCompess_Data = {
    .total = MAX_FILE_SIZE_K(2),
};
static FATCluster_Addr LogFolder_Cluster = ROOT_CLUSTER_ADDR;
static volatile Disk_FileObj_TypeDef LogFile_Obj;
static Disk_FATFileSys_TypeDef FATFS_Obj;
static bool LogFile_Ready = false;
static bool enable_compess = true;
static SrvIMU_UnionData_TypeDef LogIMU_Data __attribute__((section(".Perph_Section")));
static uint8_t LogCache_L1_Buf[MAX_FILE_SIZE_K(3)];
static uint8_t LogCache_L2_Buf[MAX_FILE_SIZE_K(3)];
static QueueObj_TypeDef IMUData_Queue;
static LogData_Reg_TypeDef LogObj_Set_Reg;
static LogData_Reg_TypeDef LogObj_Enable_Reg;
static LogData_Reg_TypeDef LogObj_Logging_Reg;
static uint16_t QueueIMU_PopSize = 0;
static uint32_t TaskLog_Period = 0;
static Log_Statistics_TypeDef Log_Statistics;

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

void TaskLog_Init(uint32_t period)
{
    memset(&Log_Statistics, 0, sizeof(Log_Statistics_TypeDef));
    memset(&IMU_Log_DataPipe, 0, sizeof(IMU_Log_DataPipe));
    memset(&LogFile_Obj, 0, sizeof(LogFile_Obj));
    memset(&FATFS_Obj, 0, sizeof(FATFS_Obj));

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
            LogFile_Obj = Disk.create_file(&FATFS_Obj, IMU_LOG_FILE, LogFolder_Cluster, MAX_FILE_SIZE_M(8));
            Disk.open(&FATFS_Obj, LOG_FOLDER, IMU_LOG_FILE, &LogFile_Obj);

            /* create cache queue for IMU Data */
            if (Queue.create_with_buf(&IMUData_Queue, "queue imu data", LogCache_L1_Buf, sizeof(LogCache_L1_Buf)))
            {
                LogFile_Ready = true;
                LogObj_Enable_Reg._sec.IMU_Sec = true;
                LogObj_Set_Reg._sec.IMU_Sec = true;
            
                DataPipe_Enable(&IMU_Log_DataPipe);

                if (lzo_init() != LZO_E_OK)
                    enable_compess = false;
            }
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
    
    TaskLog_Period = period;
}

void TaskLog_Core(void const *arg)
{
    uint8_t *compess_buf_ptr = NULL;
    uint16_t cur_compess_size = 0;
    uint16_t input_compess_size = 0;
    int ret = 0;
    uint32_t sys_time = SrvOsCommon.get_os_ms();

    while(1)
    {
        input_compess_size = QueueIMU_PopSize;
        DebugPin.ctl(Debug_PB5, true);

        if(LogFile_Ready && enable_compess)
        {
            compess_buf_ptr = LogCompess_Data.buf + (LogCompess_Data.compess_size + sizeof(uint32_t) + sizeof(uint8_t));
            cur_compess_size = 0;

            if(input_compess_size != 0)
            {
                QueueIMU_PopSize = 0;
                LogObj_Logging_Reg._sec.IMU_Sec = true;
                LogCompess_Data.buf[LogCompess_Data.compess_size] = LOG_COMPESS_HEADER;

                ret = lzo1x_1_compress(LogCache_L2_Buf, input_compess_size, compess_buf_ptr, &cur_compess_size, wrkmem);

                if(ret != LZO_E_OK)
                {
                    enable_compess = false;
                    Log_Statistics.halt_type = Log_CompessFunc_Halt;
                    DataPipe_Disable(&IMU_Log_DataPipe);
                }
                else
                {
                    if(input_compess_size <= cur_compess_size)
                    {
                        enable_compess = false;
                        Log_Statistics.halt_type = Log_CompessSize_Halt;
                        DataPipe_Disable(&IMU_Log_DataPipe);
                    }
                    else
                    {
                        /* compess count should equal to queue pop count */
                        Log_Statistics.compess_cnt++;
                        memcpy(&LogCompess_Data.buf[LogCompess_Data.compess_size + 1], &cur_compess_size, sizeof(uint32_t));

                        LogCompess_Data.compess_size = LogCompess_Data.compess_size + cur_compess_size + 5;
                        LogCompess_Data.buf[LogCompess_Data.compess_size] = LOG_COMPESS_ENDER;
                        LogCompess_Data.compess_size ++;
                        LogObj_Logging_Reg._sec.IMU_Sec = false;

                        while(LogCompess_Data.compess_size >= 512)
                        {
                            DebugPin.ctl(Debug_PB4, true);
                            if(Disk.write(&FATFS_Obj, &LogFile_Obj, LogCompess_Data.buf, 512) == Disk_Write_Finish)
                            {
                                LogFile_Ready = false;
                                Log_Statistics.halt_type = Log_Finish_Halt;
                                DataPipe_Disable(&IMU_Log_DataPipe);
                                break;
                            }
                            else
                            {
                                Log_Statistics.write_file_cnt ++;
                                Log_Statistics.log_byte_sum += 512;
                                LogCompess_Data.compess_size -= 512;

                                for(uint16_t t = 0; t < LogCompess_Data.compess_size; t++)
                                {
                                    LogCompess_Data.buf[t] = LogCompess_Data.buf[t + 512];
                                    LogCompess_Data.buf[t + 512] = 0;
                                }
                            }
                            DebugPin.ctl(Debug_PB4, false);
                        }
                    }
                }

                DevLED.ctl(Led1, true);
            }
        }
        else
            DevLED.ctl(Led1, false);

        DebugPin.ctl(Debug_PB5, false);

        SrvOsCommon.precise_delay(&sys_time, TaskLog_Period);
    }
}

static uint32_t in_queue_size = 0;
static uint32_t out_queue_size = 0;
static void TaskLog_PipeTransFinish_Callback(DataPipeObj_TypeDef *obj)
{
    uint64_t imu_pipe_rt_diff = 0;
    static uint64_t lst_imu_pipe_rt = 0;
    LogIMUDataUnion_TypeDef Log_Buf;

    if ((obj == NULL) || !LogFile_Ready)
        return;

    if (LogObj_Set_Reg._sec.IMU_Sec && (obj == &IMU_Log_DataPipe) && LogObj_Enable_Reg._sec.IMU_Sec)
    {
        if(!LogObj_Logging_Reg._sec.IMU_Sec && enable_compess && 
            Queue.size(IMUData_Queue) >= MAX_FILE_SIZE_K(1))
        {
            Log_Statistics.queue_pop_cnt ++;

            in_queue_size = Queue.size(IMUData_Queue);
            QueueIMU_PopSize = (Queue.size(IMUData_Queue) / (LOG_HEADER_SIZE + sizeof(Log_Buf))) * (LOG_HEADER_SIZE + sizeof(Log_Buf));

            /* queue pop count should equal to compess count */
            Queue.pop(&IMUData_Queue, LogCache_L2_Buf, Queue.size(IMUData_Queue));
            out_queue_size = Queue.size(IMUData_Queue);
        }

        memset(Log_Buf.data.const_res, 0, sizeof(Log_Buf.data.const_res));
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
            Log_Statistics.queue_push_err_cnt++;
        }
    }
}
