/*
 * Auther: 8_B!T0
 * this file will candidate task_log
 * drone black box task
 */
#include "FreeRTOS.h"
#include "task.h"
#include "Task_BlackBox.h"
#include "shell.h"
#include "debug_util.h"
#include "../DataStructure/CusQueue.h"
#include "error_log.h"
#include "../DataPipe/DataPipe.h"
#include "../System/storage/Storage.h"
#include "Task_Telemetry.h"
#include "Task_Sample.h"
#include "HW_Def.h"
#include "minilzo.h"
#include "Srv_OsCommon.h"
#include "Srv_BlackBox_Def.h"

#define PeriphSec __attribute__((section(".Perph_Section")))
#define BlackBox_Buff_Size (8 Kb)
#define BlackBox_Storage_Name "BlackBox"

typedef enum
{
    BlackBox_Cnv_None_Error = 0,
    BlackBox_Cnv_Para_Error,
    BlackBox_Cnv_Size_Error,
    BlackBox_Cnv_Type_Error,
    BlackBox_Cnv_Header_Error,
    BlackBox_Cnv_Ender_Error,
} BlackBox_ConvertError_List;

typedef struct
{
    uint32_t cnt;
    uint32_t byte_size;
} BlackBox_LogStatistic_TypeDef;

#pragma pack(1)
typedef struct
{
    BlackBox_MediumType_List medium;
    BlackBox_LogType_List log_type;
    uint32_t log_size;
} BlackBox_LogInfo_TypeDef;
#pragma pack()

typedef struct
{
    bool enable;
    bool write_thread_state;
    
    Storage_ItemSearchOut_TypeDef storage_search;
    BlackBox_MediumType_List medium;
    BlackBox_LogType_List log_type;
    uint32_t expection_log_size;

    uint32_t log_unit;
    uint8_t *p_log_buf;
    uint32_t log_byte_size;
    uint32_t log_cnt;
    BlackBox_LogStatistic_TypeDef imu_log;
    BlackBox_LogStatistic_TypeDef alt_att_log; 
    BlackBox_LogStatistic_TypeDef ctl_ang_log;
    BlackBox_LogStatistic_TypeDef ctl_att_log;
} BlackBox_Monitor_TypeDef;

/* internal vriable */
static uint8_t BlackBox_Buff[BlackBox_Buff_Size] PeriphSec;
static QueueObj_TypeDef BlackBox_Queue;
static BlackBox_Monitor_TypeDef Monitor;
static osSemaphoreId BlackBox_Sem = 0;
static SrvBlackBox_TypeDef *p_blackbox = NULL;
DataPipe_CreateDataObj(SrvIMU_UnionData_TypeDef,  LogImu_Data);
DataPipe_CreateDataObj(SrvBaro_UnionData_TypeDef, LogBaro_Data);
DataPipe_CreateDataObj(IMUAtt_TypeDef,            LogAtt_Data);
DataPipe_CreateDataObj(AltData_TypeDef,           LogAlt_Data);
DataPipe_CreateDataObj(ControlData_TypeDef,       LogControl_Data);

/* internal function */
static void TaskBlackBox_PipeTransFinish_Callback(DataPipeObj_TypeDef *obj);
static void TaskBlackBox_Write_Core(void const *arg);
static void TaskBlackBox_PushFinish_Callback(void);

void TaskBlackBox_Init(void)
{
    uint32_t unit = 0;
    BlackBox_LogInfo_TypeDef BlackBoxInfo;
    /* medium init */
    /* medium auto detect */
    /*
        --- TF Card
        --- W25Qxx Storage Chip
        --- Com Output
     */
    memset(&BlackBoxInfo, 0, sizeof(BlackBox_LogInfo_TypeDef));
    memset(&Monitor, 0, sizeof(BlackBox_Monitor_TypeDef));
    /* read blackbox info from storage */
    Monitor.storage_search = Storage.search(Para_User, BlackBox_Storage_Name);
    if (Monitor.storage_search.item_addr == 0)
    {
        BlackBoxInfo.log_size = 0;
        BlackBoxInfo.medium = BlackBox_Medium_None;
        BlackBoxInfo.log_type = BlackBox_Log_None;
        if (Storage.create(Para_User, BlackBox_Storage_Name, &BlackBoxInfo, sizeof(BlackBox_LogInfo_TypeDef)) != Storage_Error_None)
            return;
    }
    else
    {
        if (Storage.get(Para_User, Monitor.storage_search.item, &BlackBoxInfo, sizeof(BlackBox_LogInfo_TypeDef)) != Storage_Error_None)
            return;

        Monitor.medium = BlackBoxInfo.medium;
        Monitor.log_type = BlackBoxInfo.log_type;
        Monitor.expection_log_size = BlackBoxInfo.log_size;
    }

    Monitor.write_thread_state = false;
    switch ((uint8_t)Monitor.medium)
    {
        case BlackBox_Medium_Com:
            unit = SrvCom_BlackBox.init(TaskBlackBox_PushFinish_Callback);
            p_blackbox = &SrvCom_BlackBox;
            break;

        case BlackBox_Medium_Chip:
            unit = SrvChip_BlackBox.init(TaskBlackBox_PushFinish_Callback);
            p_blackbox = &SrvChip_BlackBox;
            break;

        case BlackBox_Medium_Card:
            unit = SrvCard_BlackBox.init(TaskBlackBox_PushFinish_Callback);
            p_blackbox = &SrvCard_BlackBox;
            break;

        default: return;
    }
    
    if (unit == 0)
    {
        p_blackbox = NULL;
        return;
    }

    switch ((uint8_t) Monitor.log_type)
    {
        case BlackBox_Imu_Filted:
        case BlackBox_Log_Alt_Att:
        case BlackBox_AngularPID_Tune:
        case BlackBox_AttitudePID_Tune:
            break;

        case BlackBox_Log_None:
        default: return;
    }

    Monitor.p_log_buf = SrvOsCommon.malloc(unit);
    if (Monitor.p_log_buf == NULL)
    {
        SrvOsCommon.free(Monitor.p_log_buf);
        return;
    }

    Monitor.log_unit = unit;
    memset(&IMU_Log_DataPipe,      0, sizeof(DataPipeObj_TypeDef));
    memset(&Baro_Log_DataPipe,     0, sizeof(DataPipeObj_TypeDef));
    memset(&Attitude_Log_DataPipe, 0, sizeof(DataPipeObj_TypeDef));
    memset(&Altitude_Log_DataPipe, 0, sizeof(DataPipeObj_TypeDef));
    memset(&CtlData_Log_DataPipe,  0, sizeof(DataPipeObj_TypeDef));

    memset(DataPipe_DataObjAddr(LogImu_Data),     0, DataPipe_DataSize(LogImu_Data));
    memset(DataPipe_DataObjAddr(LogBaro_Data),    0, DataPipe_DataSize(LogBaro_Data));
    memset(DataPipe_DataObjAddr(LogAtt_Data),     0, DataPipe_DataSize(LogAtt_Data));
    memset(DataPipe_DataObjAddr(LogAlt_Data),     0, DataPipe_DataSize(LogAlt_Data));
    memset(DataPipe_DataObjAddr(LogControl_Data), 0, DataPipe_DataSize(LogControl_Data));

    if (!Queue.create_with_buf(&BlackBox_Queue, "BlackBox_Queue", BlackBox_Buff, BlackBox_Buff_Size))
        return;

    osSemaphoreDef(Log);
    BlackBox_Sem = osSemaphoreCreate(osSemaphore(Log), 32);

    if (BlackBox_Sem == 0)
        return;

    /* pipe object init */
    IMU_Log_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(LogImu_Data);
    IMU_Log_DataPipe.data_size = DataPipe_DataSize(LogImu_Data);
    IMU_Log_DataPipe.trans_finish_cb = TaskBlackBox_PipeTransFinish_Callback;
    
    Baro_Log_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(LogBaro_Data);
    Baro_Log_DataPipe.data_size = DataPipe_DataSize(LogBaro_Data);
    Baro_Log_DataPipe.trans_finish_cb = TaskBlackBox_PipeTransFinish_Callback;

    CtlData_Log_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(LogControl_Data);
    CtlData_Log_DataPipe.data_size = DataPipe_DataSize(LogControl_Data);
    CtlData_Log_DataPipe.trans_finish_cb = TaskBlackBox_PipeTransFinish_Callback;

    Attitude_Log_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(LogAtt_Data);
    Attitude_Log_DataPipe.data_size = DataPipe_DataSize(LogAtt_Data);
    Attitude_Log_DataPipe.trans_finish_cb = TaskBlackBox_PipeTransFinish_Callback;

    Altitude_Log_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(LogAlt_Data);
    Altitude_Log_DataPipe.data_size = DataPipe_DataSize(LogAlt_Data);
    Altitude_Log_DataPipe.trans_finish_cb = TaskBlackBox_PipeTransFinish_Callback;

    DataPipe_Enable(&IMU_Log_DataPipe);
    DataPipe_Enable(&Baro_Log_DataPipe);
    DataPipe_Enable(&CtlData_Log_DataPipe);
    DataPipe_Enable(&Attitude_Log_DataPipe);
    DataPipe_Enable(&Altitude_Log_DataPipe);

    /* disable log first */
    Monitor.enable = false;
}

void TaskBlackBox_Core(void const *arg)
{
    Monitor.write_thread_state = true;

    while (BlackBox_Sem && p_blackbox)
    {
        osSemaphoreWait(BlackBox_Sem, osWaitForever);
        if (p_blackbox->push && Monitor.log_unit)
        {
            if(p_blackbox->push(Monitor.p_log_buf, Monitor.log_unit))
            {
                Monitor.log_cnt ++;
                Monitor.log_byte_size += Monitor.log_unit;
            }
        }
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

/* BlackBox Tansmit Finished Callback */
static void TaskBlackBox_PushFinish_Callback(void)
{

}

/* PIPE Callback */
static void TaskBlackBox_UpdateQueue(uint8_t *p_data, uint16_t size)
{
    uint32_t offset = 0;
    uint32_t Queue_remain = Queue.remain(BlackBox_Queue);
    
    if (size > Queue_remain)
    {
        offset = Queue_remain;
        Queue.push(&BlackBox_Queue, p_data, Queue_remain);
    }
    else
        Queue.push(&BlackBox_Queue, p_data, size);

    if (Queue.size(BlackBox_Queue) >= Monitor.log_unit)
    {
        Queue.pop(&BlackBox_Queue, Monitor.p_log_buf, Monitor.log_unit);
        osSemaphoreRelease(BlackBox_Sem);
    }

    if (offset)
        Queue.push(&BlackBox_Queue, p_data + offset, size - offset);
}

static void TaskBlackBox_PipeTransFinish_Callback(DataPipeObj_TypeDef *obj)
{
    static BlackBox_IMUData_TypeDef  imu_data;
    static BlackBox_BaroData_TypeDef baro_data;
    static BlackBox_AttAltData_TypeDef att_alt_data;
    static bool alt_update = false;
    static bool att_update = false;
    BlackBox_DataHeader_TypeDef blackbox_header;
    BlackBox_DataEnder_TypeDef blackbox_ender;
    uint8_t check_sum = 0;
    uint8_t i = 0;
    bool imu_update = false;

    memset(&blackbox_header, 0, BLACKBOX_HEADER_SIZE);
    memset(&blackbox_ender,  0, BLACKBOX_ENDER_SIZE);

    blackbox_header.header = BLACKBOX_LOG_HEADER;
    blackbox_ender.ender   = BLACKBOX_LOG_ENDER;

    if (!Monitor.enable || (Monitor.log_type == BlackBox_Log_None) || !Monitor.write_thread_state)
        return;

    /* pipe data update */
    if (obj == &IMU_Log_DataPipe)
    {
        imu_data.acc_scale = DataPipe_DataObj(LogImu_Data).data.acc_scale;
        imu_data.gyr_scale = DataPipe_DataObj(LogImu_Data).data.gyr_scale;
        for (i = Axis_X; i < Axis_Sum; i++)
        {
            imu_data.flt_gyr[i] = (int16_t)(DataPipe_DataObj(LogImu_Data).data.flt_gyr[i] * imu_data.gyr_scale);
            imu_data.org_gyr[i] = (int16_t)(DataPipe_DataObj(LogImu_Data).data.org_gyr[i] * imu_data.gyr_scale);
        
            imu_data.flt_acc[i] = (int16_t)(DataPipe_DataObj(LogImu_Data).data.flt_acc[i] * imu_data.acc_scale);
            imu_data.org_acc[i] = (int16_t)(DataPipe_DataObj(LogImu_Data).data.org_acc[i] * imu_data.acc_scale);
        }
        imu_data.time = DataPipe_DataObj(LogImu_Data).data.time_stamp;
        imu_data.cyc  = DataPipe_DataObj(LogImu_Data).data.cycle_cnt;
        imu_update = true;
    }
    else if (obj == &Baro_Log_DataPipe)
    {
        baro_data.time = DataPipe_DataObj(LogBaro_Data).data.time_stamp;
        baro_data.cyc = DataPipe_DataObj(LogBaro_Data).data.cyc;
        baro_data.press = DataPipe_DataObj(LogBaro_Data).data.pressure;
    }
    else if ((obj == &Attitude_Log_DataPipe) || \
             (obj == &Altitude_Log_DataPipe))
    {
        /* set imu data */
        att_alt_data.gyr_scale = imu_data.gyr_scale;
        att_alt_data.acc_scale = imu_data.acc_scale;
        memcpy(att_alt_data.acc, imu_data.flt_acc, sizeof(imu_data.flt_acc));
        memcpy(att_alt_data.gyr, imu_data.flt_gyr, sizeof(imu_data.flt_gyr));

        /* set baro data */
        att_alt_data.baro = baro_data.press;

        /* log data when attitude update */
        if (obj == &Attitude_Log_DataPipe)
        {
            /* set attitude */
            att_alt_data.time = DataPipe_DataObj(LogAtt_Data).time_stamp;
            att_alt_data.pitch = DataPipe_DataObj(LogAtt_Data).pitch;
            att_alt_data.roll = DataPipe_DataObj(LogAtt_Data).roll;
            att_alt_data.yaw = DataPipe_DataObj(LogAtt_Data).yaw;
            att_update = true;
        }
        else if (obj == &Altitude_Log_DataPipe)
        {
            /* set altitude */
            att_alt_data.alt = DataPipe_DataObj(LogAlt_Data).alt;
            alt_update = true;
        }
    }

    /* blackbox log control */
    if ((Monitor.log_type == BlackBox_Imu_Filted) && imu_update)
    {
        /* log data when imu update */
        Monitor.imu_log.cnt ++;
        Monitor.imu_log.byte_size += BLACKBOX_HEADER_SIZE;
        blackbox_header.type = BlackBox_Imu_Filted;
        blackbox_header.size = sizeof(BlackBox_IMUData_TypeDef);
        TaskBlackBox_UpdateQueue((uint8_t *)&blackbox_header, BLACKBOX_HEADER_SIZE);
        
        TaskBlackBox_UpdateQueue((uint8_t *)&imu_data, sizeof(BlackBox_IMUData_TypeDef));
        Monitor.imu_log.byte_size += sizeof(BlackBox_IMUData_TypeDef);

        check_sum = TaskBlackBox_Get_CheckSum((uint8_t *)&imu_data, sizeof(BlackBox_IMUData_TypeDef));
        blackbox_ender.check_sum = check_sum;
        Monitor.imu_log.byte_size += BLACKBOX_ENDER_SIZE;
        TaskBlackBox_UpdateQueue((uint8_t *)&blackbox_ender, BLACKBOX_ENDER_SIZE);
        imu_update = false;
    }
    else if ((Monitor.log_type == BlackBox_Log_Alt_Att) && (alt_update & att_update & imu_update))
    {
        Monitor.alt_att_log.cnt ++;
        Monitor.alt_att_log.byte_size += BLACKBOX_HEADER_SIZE;
        blackbox_header.type = BlackBox_Log_Alt_Att;
        blackbox_header.size = sizeof(BlackBox_AttAltData_TypeDef);
        TaskBlackBox_UpdateQueue((uint8_t *)&blackbox_header, BLACKBOX_HEADER_SIZE);
        
        TaskBlackBox_UpdateQueue((uint8_t *)&att_alt_data, sizeof(BlackBox_AttAltData_TypeDef));
        Monitor.alt_att_log.byte_size += sizeof(BlackBox_AttAltData_TypeDef);

        check_sum = TaskBlackBox_Get_CheckSum((uint8_t *)&att_alt_data, sizeof(BlackBox_AttAltData_TypeDef));
        blackbox_ender.check_sum = check_sum;
        Monitor.imu_log.byte_size += BLACKBOX_ENDER_SIZE;
        TaskBlackBox_UpdateQueue((uint8_t *)&blackbox_ender, BLACKBOX_ENDER_SIZE);
        alt_update = false;
        att_update = false;
    }
    else if (Monitor.log_type == BlackBox_AngularPID_Tune)
    {

    }
    else if (Monitor.log_type == BlackBox_AttitudePID_Tune)
    {

    }

    /* use minilzo compress data if have to */
}

void TaskBlackBox_LogControl(void)
{
    if (!Monitor.enable)
    {
        if (p_blackbox->enable && p_blackbox->enable())
            Monitor.enable = true;
    }
    else
    {
        Monitor.enable = false;
        if (p_blackbox->disable)
            p_blackbox->disable();
    }
}

bool TaskBlackBox_Set_LogInfo(BlackBox_MediumType_List medium, BlackBox_LogType_List type, uint32_t size)
{
    BlackBox_LogInfo_TypeDef info;
    Storage_ItemSearchOut_TypeDef search_out;

    memset(&search_out, 0, sizeof(Storage_ItemSearchOut_TypeDef));
    memset(&info, 0, sizeof(BlackBox_LogInfo_TypeDef));

    switch ((uint8_t) medium)
    {
        case BlackBox_Medium_Card:
        case BlackBox_Medium_Chip:
        case BlackBox_Medium_Com:
            break;
        default: return false;
    }

    switch ((uint8_t) type)
    {
        case BlackBox_Log_None:
        case BlackBox_Imu_Filted:
        case BlackBox_Log_Alt_Att:
        case BlackBox_AngularPID_Tune:
        case BlackBox_AttitudePID_Tune:
            break;
        default: return false;
    }

    info.medium = medium;
    info.log_type = type;
    info.log_size = size;

    search_out = Storage.search(Para_User, BlackBox_Storage_Name);
    if (search_out.item_addr == 0)
    {
        /* create storage section */
        if (Storage.create(Para_User, BlackBox_Storage_Name, (uint8_t *)&info, sizeof(BlackBox_LogInfo_TypeDef)) != Storage_Error_None)
            return false;
    }

    if (Storage.update(Para_User, search_out.item.data_addr, (uint8_t *)&info, sizeof(BlackBox_LogInfo_TypeDef)) != Storage_Erase_Error)
        return false;

    return true;
}

/****************************************************************** shell api implement ***************************************************** */
static void TaskBlackBox_EnableLog(void)
{
    Shell *shell_obj = Shell_GetInstence();

    if ((shell_obj == NULL) || (p_blackbox == NULL))
        return;

    if (p_blackbox->enable && p_blackbox->enable())
    {
        shellPrint(shell_obj, "[ BalckBox ] Enable\r\r");
        Monitor.enable = true;
    }
    else
        shellPrint(shell_obj, "[ BlackBox ] Enable Failed\r\n");
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, blackbox_enable, TaskBlackBox_EnableLog, blackbox start log);

static void TaskBlackBox_DisableLog(void)
{
    Shell *shell_obj = Shell_GetInstence();

    if ((shell_obj == NULL) || (p_blackbox == NULL))
        return;

    if (p_blackbox->disable && p_blackbox->disable())
    {
        shellPrint(shell_obj, "[ BalckBox ] Disable\r\r");
        Monitor.enable = false;
    }
    else
        shellPrint(shell_obj, "[ BlackBox ] Disable Failed\r\n");
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, blackbox_disable, TaskBlackBox_DisableLog, blackbox end log);

static BlackBox_ConvertError_List TaskBlackBox_ConvertLogData_To_Header(Shell *p_shell, BlackBox_DataHeader_TypeDef *p_header, uint8_t **p_data, uint32_t *len)
{
    if ((p_header == NULL) || \
        (p_data == NULL) || \
        (len == NULL) || \
        (*p_data == NULL))
        return BlackBox_Cnv_Para_Error;
    
    if (*len <= BLACKBOX_HEADER_SIZE)
        return BlackBox_Cnv_Size_Error;

    memcpy(p_header, *p_data, BLACKBOX_HEADER_SIZE);
    if (p_header->header != BLACKBOX_LOG_HEADER)
    {
        memset(p_header, 0, BLACKBOX_HEADER_SIZE);
        shellPrint(p_shell, "[ BlackBox ] header tag error\r\n");
        return BlackBox_Cnv_Header_Error;
    }

    switch ((uint8_t) p_header->type)
    {
        case BlackBox_Imu_Filted:
            if (*len < (BLACKBOX_HEADER_SIZE + BLACKBOX_ENDER_SIZE + sizeof(BlackBox_IMUData_TypeDef)))
                return BlackBox_Cnv_Size_Error;
            break;

        case BlackBox_Log_Alt_Att:
            if (*len < (BLACKBOX_HEADER_SIZE + BLACKBOX_ENDER_SIZE + sizeof(BlackBox_AttAltData_TypeDef)))
                return BlackBox_Cnv_Size_Error;
            break;

        default:
            if (p_shell)
                shellPrint(p_shell, "[ BlackBox ] type error\r\n");
            return BlackBox_Cnv_Type_Error;
    }

    *len -= BLACKBOX_HEADER_SIZE;
    *p_data += BLACKBOX_HEADER_SIZE;

    return BlackBox_Cnv_None_Error;
}

static BlackBox_ConvertError_List TaskBlackBox_ConvertLogData_To_Ender(Shell *p_shell, BlackBox_DataEnder_TypeDef *p_ender, uint8_t **p_data, uint32_t *len)
{
    if ((p_ender == NULL) || \
        (p_data == NULL) || \
        (*p_data == NULL) || \
        (len == NULL) || \
        (*len < BLACKBOX_ENDER_SIZE))
        return BlackBox_Cnv_Para_Error;

    memcpy(p_ender, *p_data, BLACKBOX_ENDER_SIZE);
    if (p_ender->ender != BLACKBOX_LOG_ENDER)
    {
        memset(p_ender, 0, BLACKBOX_ENDER_SIZE);
        if (p_shell)
            shellPrint(p_shell, "[ BlackBox ] ender error\r\n");
        return BlackBox_Cnv_Ender_Error;
    }

    *p_data += BLACKBOX_ENDER_SIZE;
    *len -= BLACKBOX_ENDER_SIZE;

    return BlackBox_Cnv_None_Error;
}

/* filted imu data log */
static void TaskBlackBox_ConvertLogData_To_IMU(Shell *p_shell, BlackBox_IMUData_TypeDef *p_imu, uint8_t **p_data, uint32_t *len)
{
    if ((p_imu == NULL) || \
        (p_data == NULL) || \
        (*p_data == NULL) || \
        (len == NULL) || \
        (*len < sizeof(BlackBox_IMUData_TypeDef)))
        return;

    memcpy(p_imu, *p_data, sizeof(BlackBox_IMUData_TypeDef));
    if (p_shell)
    {
        shellPrint(p_shell, "%d ", p_imu->time);
        shellPrint(p_shell, "%d ", p_imu->cyc);
        shellPrint(p_shell, "%f ", p_imu->org_acc[Axis_X] / p_imu->acc_scale);
        shellPrint(p_shell, "%f ", p_imu->org_acc[Axis_Y] / p_imu->acc_scale);
        shellPrint(p_shell, "%f ", p_imu->org_acc[Axis_Z] / p_imu->acc_scale);
        shellPrint(p_shell, "%f ", p_imu->flt_acc[Axis_X] / p_imu->acc_scale);
        shellPrint(p_shell, "%f ", p_imu->flt_acc[Axis_Y] / p_imu->acc_scale);
        shellPrint(p_shell, "%f ", p_imu->flt_acc[Axis_Z] / p_imu->acc_scale);
        shellPrint(p_shell, "%f ", p_imu->org_gyr[Axis_X] / p_imu->gyr_scale);
        shellPrint(p_shell, "%f ", p_imu->org_gyr[Axis_Y] / p_imu->gyr_scale);
        shellPrint(p_shell, "%f ", p_imu->org_gyr[Axis_Z] / p_imu->gyr_scale);
        shellPrint(p_shell, "%f ", p_imu->flt_gyr[Axis_X] / p_imu->gyr_scale);
        shellPrint(p_shell, "%f ", p_imu->flt_gyr[Axis_Y] / p_imu->gyr_scale);
        shellPrint(p_shell, "%f\r\n", p_imu->flt_gyr[Axis_Z] / p_imu->gyr_scale);
    }

    *p_data += sizeof(BlackBox_IMUData_TypeDef);
    *len -= sizeof(BlackBox_IMUData_TypeDef);
}

/* attitude altitude log data */
static void TaskBlackBox_ConverLogData_To_Alt_Att(Shell *p_shell, BlackBox_AttAltData_TypeDef *p_att_alt, uint8_t **p_data, uint32_t *len)
{
    if ((p_att_alt == NULL) || \
        (p_data == NULL) || \
        (*p_data == NULL) || \
        (len == NULL) || \
        (*len < sizeof(BlackBox_AttAltData_TypeDef)))
        return;
}

/* angular control log data */
static void TaskBlackBox_ConvertLogData_To_CtlAng(Shell *p_shell, BlackBox_AngCtlData_TypeDef *p_ang_ctl, uint8_t **p_data, uint32_t *len)
{
    if ((p_ang_ctl == NULL) || \
        (p_data == NULL) || \
        (*p_data == NULL) || \
        (len == NULL) || \
        (*len < sizeof(BlackBox_AngCtlData_TypeDef)))
        return;
}

/* attitude control log data */
static void TaskBlackBox_ConvertLogData_To_CtlAtt(Shell *p_shell, BlackBox_AttCtlData_TypeDef *p_att_ctl, uint8_t **p_data, uint32_t *len)
{
    if ((p_att_ctl == NULL) || \
        (p_data == NULL) || \
        (*p_data == NULL) || \
        (len == NULL) || \
        (*len < sizeof(BlackBox_AttCtlData_TypeDef)))
        return;
}

static void TaskBlackBox_GetLogInfo(void)
{
    Shell *shell_obj = Shell_GetInstence();
    uint32_t log_cnt = 0;
    uint32_t log_size = 0;
    bool log_enable = false;
    uint32_t addr = 0;
    volatile BlackBox_ConvertError_List err = BlackBox_Cnv_None_Error;
    BlackBox_DataHeader_TypeDef header;
    BlackBox_DataEnder_TypeDef ender;
    BlackBox_IMUData_TypeDef log_imu;
    uint8_t *p_log_data = NULL;
    uint32_t uncomplete_size = 0;
    uint8_t remain_buf[128];
    uint8_t *p_remain_buf = NULL;
    uint8_t check_sum = 0;

    if ((shell_obj == NULL) || (p_blackbox == NULL))
        return;

    if (p_blackbox->get_info == NULL)
    {
        shellPrint(shell_obj, "[ BlackBox ] get info api error\r\n");
        return;
    }

    p_blackbox->get_info(&log_cnt, &log_size, &log_enable);
    if (log_enable)
    {
        shellPrint(shell_obj, "[ BlackBox ] is logging\r\n");
        return;
    }

    shellPrint(shell_obj, "[ BlackBox ] task log success count: %d\r\n", Monitor.log_cnt);
    shellPrint(shell_obj, "[ BlackBox ] service log count:      %d\r\n", log_cnt);
    shellPrint(shell_obj, "[ BlackBox ] service log size:       %d\r\n", log_size);

    /* dispaly data */
    if ((p_blackbox->read == NULL) || (Monitor.p_log_buf == NULL))
    {
        shellPrint(shell_obj, "[ BlackBox ] read api error\r\n");
        return;
    }

    memset(&header, 0, BLACKBOX_HEADER_SIZE);
    for (uint16_t i = 0; i < log_cnt; i++)
    {
        if (p_blackbox->read(addr, Monitor.p_log_buf, Monitor.log_unit))
        {
            log_size = Monitor.log_unit;
            p_log_data = Monitor.p_log_buf;
            addr += Monitor.log_unit;

            /* convert data */
            while (log_size)
            {
                if (uncomplete_size)
                {
                    p_remain_buf = remain_buf;

                    if (uncomplete_size < BLACKBOX_HEADER_SIZE)
                    {
                        /* get header first */
                        memcpy(p_remain_buf, p_log_data, BLACKBOX_HEADER_SIZE - uncomplete_size);
                        p_log_data += (BLACKBOX_HEADER_SIZE - uncomplete_size);
                        log_size -= (BLACKBOX_HEADER_SIZE - uncomplete_size);
                        uncomplete_size = BLACKBOX_HEADER_SIZE;
                        err = TaskBlackBox_ConvertLogData_To_Header(shell_obj, &header, &p_remain_buf, &uncomplete_size);
                        uncomplete_size = 0;
                    }
                    else if (uncomplete_size >= BLACKBOX_HEADER_SIZE)
                    {
                        err = TaskBlackBox_ConvertLogData_To_Header(shell_obj, &header, &p_remain_buf, &uncomplete_size);
                        if (err != BlackBox_Cnv_None_Error)
                        {
                            shellPrint(shell_obj, "[ BlackBox ] uncomplete decode error\r\n");
                            return;
                        }

                        if (uncomplete_size)
                        {
                            uint32_t decode_size = 0;
                            switch ((uint8_t)header.type)
                            {
                                case BlackBox_Imu_Filted:
                                    if (uncomplete_size <= sizeof(BlackBox_IMUData_TypeDef))
                                    {
                                        memcpy(p_remain_buf + uncomplete_size, p_log_data, sizeof(BlackBox_IMUData_TypeDef) - uncomplete_size);
                                        p_log_data += sizeof(BlackBox_IMUData_TypeDef) - uncomplete_size;
                                        log_size -= sizeof(BlackBox_IMUData_TypeDef) - uncomplete_size;
                                        uncomplete_size = 0;
                                    }
                                    else
                                    {
                                        memmove(p_remain_buf, p_remain_buf + uncomplete_size, uncomplete_size - sizeof(BlackBox_IMUData_TypeDef));
                                        uncomplete_size -= sizeof(BlackBox_IMUData_TypeDef);
                                    }

                                    decode_size = sizeof(BlackBox_IMUData_TypeDef);
                                    TaskBlackBox_ConvertLogData_To_IMU(shell_obj, &log_imu, &p_remain_buf, &decode_size);
                                    break;

                                case BlackBox_Log_Alt_Att:
                                    break;

                                default: return;
                            }

                            /* get ender */
                            if (uncomplete_size)
                            {
                                memcpy(remain_buf + uncomplete_size, p_log_data, BLACKBOX_ENDER_SIZE - uncomplete_size);
                                decode_size = BLACKBOX_ENDER_SIZE;
                                p_log_data += BLACKBOX_ENDER_SIZE - uncomplete_size;
                                log_size -= BLACKBOX_ENDER_SIZE - uncomplete_size;
                                TaskBlackBox_ConvertLogData_To_Ender(shell_obj, &ender, &p_remain_buf, &decode_size);
                            }
                            else
                                TaskBlackBox_ConvertLogData_To_Ender(shell_obj, &ender, &p_log_data, &log_size);

                            /* get next header */
                            err = TaskBlackBox_ConvertLogData_To_Header(shell_obj, &header, &p_log_data, &log_size);
                        }
                    }
                }
                else
                    err = TaskBlackBox_ConvertLogData_To_Header(shell_obj, &header, &p_log_data, &log_size);

                if (err == BlackBox_Cnv_None_Error)
                {
                    uncomplete_size = 0;
                    switch ((uint8_t)header.type)
                    {
                        case BlackBox_Imu_Filted:
                            TaskBlackBox_ConvertLogData_To_IMU(shell_obj, &log_imu, &p_log_data, &log_size);
                            check_sum = TaskBlackBox_Get_CheckSum((uint8_t *)&log_imu, sizeof(BlackBox_IMUData_TypeDef));
                            break;

                        default: break;
                    }

                    if (TaskBlackBox_ConvertLogData_To_Ender(shell_obj, &ender, &p_log_data, &log_size) != BlackBox_Cnv_None_Error)
                        return;

                    if (check_sum != ender.check_sum)
                    {
                        shellPrint(shell_obj, "[ BlackBox ] CheckSum error\r\n");
                        shellPrint(shell_obj, "[ BlackBox ] addr: %d remian: %d\r\n", addr, log_size);
                    }
                }
                else if (err == BlackBox_Cnv_Size_Error)
                {
                    memset(remain_buf, 0, sizeof(remain_buf));
                    uncomplete_size = log_size;
                    memcpy(remain_buf, p_log_data, uncomplete_size);
                    log_size = 0;
                }
                else
                    break;
            }
        }
        else
        {
            shellPrint(shell_obj, "[ BlackBox ] Data read failed\r\n");
            shellPrint(shell_obj, "[ BlackBox ] read address %d\r\n", addr);
            break;
        }
    }
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC) | SHELL_CMD_DISABLE_RETURN, blackbox_info, TaskBlackBox_GetLogInfo, blackbox log info);
