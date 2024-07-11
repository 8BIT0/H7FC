#include "Task_BlackBox.h"
#include "shell.h"
#include "debug_util.h"
#include "../DataStructure/CusQueue.h"
#include "error_log.h"
#include "../DataPipe/DataPipe.h"
#include "Task_Telemetry.h"
#include "Task_Sample.h"
#include "HW_Def.h"
#include "minilzo.h"
#include "Srv_OsCommon.h"

typedef struct
{
    uint32_t time;
    uint32_t cnt;
    uint32_t byte_size;
} BlackBox_LogMonitor_TypeDef;

#define PeriphSec __attribute__((section(".Perph_Section")))
#define BlackBox_Buff_Size (8 Kb)

/* internal vriable */
static uint8_t BlackBox_Buff[BlackBox_Buff_Size] PeriphSec;
static QueueObj_TypeDef Data_Queue;
static osSemaphoreId BlackBox_Sem;
DataPipe_CreateDataObj(SrvIMU_UnionData_TypeDef,  LogImu_Data);
DataPipe_CreateDataObj(SrvBaro_UnionData_TypeDef, LogBaro_Data);
DataPipe_CreateDataObj(ControlData_TypeDef,       LogControl_Data);

static BlackBox_LogMonitor_TypeDef imu_log;
static BlackBox_LogMonitor_TypeDef baro_log; 
static BlackBox_LogMonitor_TypeDef ctl_log;
static BlackBox_LogMonitor_TypeDef act_log;
static BlackBox_LogMonitor_TypeDef att_log;

static uint32_t log_byte_size = 0;

/* internal function */
static void TaskBlackBox_PipeTransFinish_Callback(DataPipeObj_TypeDef *obj);

void TaskBlackBox_Init(void)
{
    memset(&imu_log,  0, sizeof(BlackBox_LogMonitor_TypeDef));
    memset(&baro_log, 0, sizeof(BlackBox_LogMonitor_TypeDef));
    memset(&ctl_log,  0, sizeof(BlackBox_LogMonitor_TypeDef));
    memset(&act_log,  0, sizeof(BlackBox_LogMonitor_TypeDef));
    memset(&att_log,  0, sizeof(BlackBox_LogMonitor_TypeDef));

    memset(&IMU_Log_DataPipe,      0, sizeof(DataPipeObj_TypeDef));
    memset(&Baro_Log_DataPipe,     0, sizeof(DataPipeObj_TypeDef));
    memset(&Attitude_Log_DataPipe, 0, sizeof(DataPipeObj_TypeDef));
    memset(&Actuator_Log_DataPipe, 0, sizeof(DataPipeObj_TypeDef));
    memset(&CtlData_Log_DataPipe,  0, sizeof(DataPipeObj_TypeDef));

    memset(DataPipe_DataObjAddr(LogImu_Data),     0, DataPipe_DataSize(LogImu_Data));
    memset(DataPipe_DataObjAddr(LogBaro_Data),    0, DataPipe_DataSize(LogBaro_Data));
    memset(DataPipe_DataObjAddr(LogControl_Data), 0, DataPipe_DataSize(LogControl_Data));

    if (!Queue.create_with_buf(&Data_Queue, "BlackBox_Queue", BlackBox_Buff, BlackBox_Buff_Size))
        return;

    osSemaphoreDef(Log);
    BlackBox_Sem = osSemaphoreCreate(osSemaphore(Log), 1);

    /* pipe object init */
    IMU_Log_DataPipe.data_addr = DataPipe_DataObjAddr(LogImu_Data);
    IMU_Log_DataPipe.data_size = DataPipe_DataSize(LogImu_Data);
    IMU_Log_DataPipe.trans_finish_cb = TaskBlackBox_PipeTransFinish_Callback;
    
    Baro_Log_DataPipe.data_addr = DataPipe_DataObjAddr(LogBaro_Data);
    Baro_Log_DataPipe.data_size = DataPipe_DataSize(LogBaro_Data);
    Baro_Log_DataPipe.trans_finish_cb = TaskBlackBox_PipeTransFinish_Callback;

    CtlData_Log_DataPipe.data_addr = DataPipe_DataObjAddr(LogControl_Data);
    CtlData_Log_DataPipe.data_size = DataPipe_DataSize(LogControl_Data);
    CtlData_Log_DataPipe.trans_finish_cb = TaskBlackBox_PipeTransFinish_Callback;

    DataPipe_Enable(&IMU_Log_DataPipe);
    DataPipe_Enable(&Baro_Log_DataPipe);
    DataPipe_Enable(&CtlData_Log_DataPipe);
    // DataPipe_Enable(&Attitude_Log_DataPipe);
    // DataPipe_Enable(&Actuator_Log_DataPipe);
}

void TaskBlackBox_Core(void const *arg)
{
    while (BlackBox_Sem)
    {
        osSemaphoreWait(BlackBox_Sem, osWaitForever);
    }
}

static uint8_t TaskBlackBox_Get_CheckSum(uint8_t *p_data, uint16_t len)
{
    uint8_t check_sum = 0;

    if (p_data && len)
        for (uint16_t i = 0; i < len; i++)
            check_sum += p_data[i];

    return check_sum;
}

/* PIPE Callback */
static void TaskBlackBox_PipeTransFinish_Callback(DataPipeObj_TypeDef *obj)
{
    BlackBox_DataHeader_TypeDef   blackbox_header;
    BlackBox_DataEnder_TypeDef    blackbox_ender;
    BlackBox_IMUData_TypeDef      imu_data;
    BlackBox_BaroData_TypeDef     baro_data;
    BlackBox_CtlData_TypeDef      input_ctl_data;
    BlackBox_AttitudeData_TypeDef att_data;
    BlackBox_ActuatorData_TypeDef actuator_data;
    uint8_t check_sum = 0;

    memset(&blackbox_header, 0, BLACKBOX_HEADER_SIZE);
    memset(&blackbox_ender,  0, BLACKBOX_ENDER_SIZE);

    blackbox_header.header = BLACKBOX_LOG_HEADER;
    blackbox_ender.ender   = BLACKBOX_LOG_ENDER;

    if (obj == &IMU_Log_DataPipe)
    {
        memset(&imu_data, 0, sizeof(BlackBox_IMUData_TypeDef));
        imu_log.cnt ++;
        imu_log.byte_size += BLACKBOX_HEADER_SIZE;
        blackbox_header.type = BlackBox_IMU;
        
        imu_data.acc_scale = DataPipe_DataObj(LogImu_Data).data.acc_scale;
        imu_data.gyr_scale = DataPipe_DataObj(LogImu_Data).data.gyr_scale;
        for (uint8_t i = Axis_X; i < Axis_Sum; i++)
        {
            imu_data.flt_acc[i] = (uint16_t)(DataPipe_DataObj(LogImu_Data).data.flt_gyr[i] * imu_data.gyr_scale);
            imu_data.org_gyr[i] = (uint16_t)(DataPipe_DataObj(LogImu_Data).data.org_gyr[i] * imu_data.gyr_scale);
        
            imu_data.flt_acc[i] = (uint16_t)(DataPipe_DataObj(LogImu_Data).data.flt_acc[i] * imu_data.acc_scale);
            imu_data.org_acc[i] = (uint16_t)(DataPipe_DataObj(LogImu_Data).data.org_acc[i] * imu_data.acc_scale);
        }
        imu_data.time = DataPipe_DataObj(LogImu_Data).data.time_stamp;
        imu_data.cyc  = DataPipe_DataObj(LogImu_Data).data.cycle_cnt;
        imu_log.byte_size += sizeof(BlackBox_IMUData_TypeDef);

        check_sum = TaskBlackBox_Get_CheckSum(&imu_data, sizeof(imu_data));
        blackbox_ender.check_sum = check_sum;
        imu_log.byte_size += BLACKBOX_HEADER_SIZE;
    }
    
    if (obj == &Baro_Log_DataPipe)
    {
        memset(&baro_data, 0, sizeof(BlackBox_BaroData_TypeDef));
        baro_log.cnt ++;
        baro_log.byte_size += BLACKBOX_HEADER_SIZE;
        blackbox_header.type = BlackBox_Baro;
        baro_data.time = DataPipe_DataObj(LogBaro_Data).data.time_stamp;
        baro_data.cyc = DataPipe_DataObj(LogBaro_Data).data.cyc;
        baro_data.press = DataPipe_DataObj(LogBaro_Data).data.pressure;
        baro_log.byte_size += sizeof(BlackBox_BaroData_TypeDef);

        check_sum = TaskBlackBox_Get_CheckSum(&baro_data, sizeof(BlackBox_BaroData_TypeDef));
        blackbox_ender.check_sum = check_sum;
        baro_log.byte_size += BLACKBOX_ENDER_SIZE;
    }
    
    if (obj == &CtlData_Log_DataPipe)
    {
        memset(&input_ctl_data, 0, sizeof(BlackBox_CtlData_TypeDef));
        ctl_log.cnt ++;
        ctl_log.byte_size += BLACKBOX_HEADER_SIZE;
        blackbox_header.type = BlackBox_CtlData;
        ctl_log.byte_size += BLACKBOX_ENDER_SIZE;
    }
    
    if (obj == &Attitude_Log_DataPipe)
    {
        memset(&att_data, 0, sizeof(BlackBox_AttitudeData_TypeDef));
        att_log.cnt ++;
        att_log.byte_size += BLACKBOX_HEADER_SIZE;
        blackbox_header.type = BlackBox_Attitude;
        att_log.byte_size += BLACKBOX_ENDER_SIZE;
    }
    
    if (obj == & Actuator_Log_DataPipe)
    {
        memset(&actuator_data, 0, sizeof(BlackBox_ActuatorData_TypeDef));
        act_log.cnt ++;
        act_log.byte_size += BLACKBOX_HEADER_SIZE;
        blackbox_header.type = BlackBox_Actuator;
        act_log.byte_size += BLACKBOX_ENDER_SIZE;
    }
}



