/*
 * Auther: 8_B!T0
 * this file is the candidate of task_log
 */
#include "FreeRTOS.h"
#include "task.h"
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
#include "Srv_BlackBox_Def.h"

#define PeriphSec __attribute__((section(".Perph_Section")))
#define BlackBox_Buff_Size (8 Kb)

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
} BlackBox_LogMonitor_TypeDef;

typedef struct
{
    bool enable;
    bool write_thread_state;

    BlackBox_MediumType_List medium;
    BlackBox_Reg_TypeDef en_reg;

    uint32_t log_unit;
    uint8_t *p_log_buf;
    uint32_t log_byte_size;
    uint32_t log_cnt;
    BlackBox_LogMonitor_TypeDef imu_log;
    BlackBox_LogMonitor_TypeDef baro_log; 
    BlackBox_LogMonitor_TypeDef ctl_log;
    BlackBox_LogMonitor_TypeDef act_log;
    BlackBox_LogMonitor_TypeDef att_log;
} BlackBox_Monitor_TypeDef;

/* internal vriable */
static uint8_t BlackBox_Buff[BlackBox_Buff_Size] PeriphSec;
static QueueObj_TypeDef BlackBox_Queue;
static osSemaphoreId BlackBox_Sem;
static BlackBox_Monitor_TypeDef Monitor;
static SrvBlackBox_TypeDef *p_blackbox = NULL;
DataPipe_CreateDataObj(SrvIMU_UnionData_TypeDef,  LogImu_Data);
DataPipe_CreateDataObj(SrvBaro_UnionData_TypeDef, LogBaro_Data);
DataPipe_CreateDataObj(ControlData_TypeDef,       LogControl_Data);

/* internal function */
static void TaskBlackBox_PipeTransFinish_Callback(DataPipeObj_TypeDef *obj);
static void TaskBlackBox_Write_Core(void const *arg);
static void TaskBlackBox_PushFinish_Callback(void);

void TaskBlackBox_Init(void)
{
    uint32_t unit = 0;
    /* medium init */
    /* medium auto detect */
    /*
        --- TF Card
        --- W25Qxx Storage Chip
        --- Com Output
     */
    memset(&Monitor, 0, sizeof(BlackBox_Monitor_TypeDef));
    Monitor.write_thread_state = false;
    Monitor.medium = BlackBox_Medium_Chip;
    Monitor.en_reg.val = 0;
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

    Monitor.p_log_buf = SrvOsCommon.malloc(unit);
    if (Monitor.p_log_buf == NULL)
    {
        SrvOsCommon.free(Monitor.p_log_buf);
        return;
    }

    Monitor.log_unit = unit;
    Monitor.en_reg.bit.imu = true;
    Monitor.en_reg.bit.baro = true;
    Monitor.en_reg.bit.exp_ctl = false;

    memset(&IMU_Log_DataPipe,      0, sizeof(DataPipeObj_TypeDef));
    memset(&Baro_Log_DataPipe,     0, sizeof(DataPipeObj_TypeDef));
    memset(&Attitude_Log_DataPipe, 0, sizeof(DataPipeObj_TypeDef));
    memset(&Actuator_Log_DataPipe, 0, sizeof(DataPipeObj_TypeDef));
    memset(&CtlData_Log_DataPipe,  0, sizeof(DataPipeObj_TypeDef));

    memset(DataPipe_DataObjAddr(LogImu_Data),     0, DataPipe_DataSize(LogImu_Data));
    memset(DataPipe_DataObjAddr(LogBaro_Data),    0, DataPipe_DataSize(LogBaro_Data));
    memset(DataPipe_DataObjAddr(LogControl_Data), 0, DataPipe_DataSize(LogControl_Data));

    if (!Queue.create_with_buf(&BlackBox_Queue, "BlackBox_Queue", BlackBox_Buff, BlackBox_Buff_Size))
        return;

    osSemaphoreDef(Log);
    BlackBox_Sem = osSemaphoreCreate(osSemaphore(Log), 32);

    if (BlackBox_Sem == NULL)
        return;

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

    /* disable log first */
    Monitor.enable = false;
}

void TaskBlackBox_Core(void const *arg)
{
    Monitor.write_thread_state = true;
    volatile uint32_t time_1 = 0;
    volatile uint32_t time_2 = 0;

    while (BlackBox_Sem && p_blackbox)
    {
        osSemaphoreWait(BlackBox_Sem, osWaitForever);
        if (p_blackbox->push && Monitor.log_unit)
        {
            time_1 = SrvOsCommon.get_os_ms();
            if(p_blackbox->push(Monitor.p_log_buf, Monitor.log_unit))
            {
                time_2 = SrvOsCommon.get_os_ms();
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

    if (!Monitor.enable || !Monitor.write_thread_state)
        return;

    if ((obj == &IMU_Log_DataPipe) && Monitor.en_reg.bit.imu)
    {
        memset(&imu_data, 0, sizeof(BlackBox_IMUData_TypeDef));
        Monitor.imu_log.cnt ++;
        Monitor.imu_log.byte_size += BLACKBOX_HEADER_SIZE;
        blackbox_header.type = BlackBox_IMU;
        blackbox_header.size = sizeof(BlackBox_IMUData_TypeDef);
        TaskBlackBox_UpdateQueue((uint8_t *)&blackbox_header, BLACKBOX_HEADER_SIZE);
        
        imu_data.acc_scale = DataPipe_DataObj(LogImu_Data).data.acc_scale;
        imu_data.gyr_scale = DataPipe_DataObj(LogImu_Data).data.gyr_scale;
        for (uint8_t i = Axis_X; i < Axis_Sum; i++)
        {
            imu_data.flt_gyr[i] = (int16_t)(DataPipe_DataObj(LogImu_Data).data.flt_gyr[i] * imu_data.gyr_scale);
            // imu_data.org_gyr[i] = (int16_t)(DataPipe_DataObj(LogImu_Data).data.org_gyr[i] * imu_data.gyr_scale);
        
            imu_data.flt_acc[i] = (int16_t)(DataPipe_DataObj(LogImu_Data).data.flt_acc[i] * imu_data.acc_scale);
            // imu_data.org_acc[i] = (int16_t)(DataPipe_DataObj(LogImu_Data).data.org_acc[i] * imu_data.acc_scale);
        }
        imu_data.time = DataPipe_DataObj(LogImu_Data).data.time_stamp;
        imu_data.cyc  = DataPipe_DataObj(LogImu_Data).data.cycle_cnt;
        TaskBlackBox_UpdateQueue((uint8_t *)&imu_data, sizeof(BlackBox_IMUData_TypeDef));
        Monitor.imu_log.byte_size += sizeof(BlackBox_IMUData_TypeDef);

        check_sum = TaskBlackBox_Get_CheckSum(&imu_data, sizeof(imu_data));
        blackbox_ender.check_sum = check_sum;
        Monitor.imu_log.byte_size += BLACKBOX_ENDER_SIZE;
        TaskBlackBox_UpdateQueue((uint8_t *)&blackbox_ender, BLACKBOX_ENDER_SIZE);
    }

    if ((obj == &Baro_Log_DataPipe) && Monitor.en_reg.bit.baro)
    {
        memset(&baro_data, 0, sizeof(BlackBox_BaroData_TypeDef));
        Monitor.baro_log.cnt ++;
        Monitor.baro_log.byte_size += BLACKBOX_HEADER_SIZE;
        blackbox_header.type = BlackBox_Baro;
        blackbox_header.size = sizeof(BlackBox_BaroData_TypeDef);
        TaskBlackBox_UpdateQueue((uint8_t *)&blackbox_header, BLACKBOX_HEADER_SIZE);
        
        baro_data.time = DataPipe_DataObj(LogBaro_Data).data.time_stamp;
        baro_data.cyc = DataPipe_DataObj(LogBaro_Data).data.cyc;
        baro_data.press = DataPipe_DataObj(LogBaro_Data).data.pressure;
        TaskBlackBox_UpdateQueue((uint8_t *)&baro_data, sizeof(BlackBox_BaroData_TypeDef));
        Monitor.baro_log.byte_size += sizeof(BlackBox_BaroData_TypeDef);

        check_sum = TaskBlackBox_Get_CheckSum(&baro_data, sizeof(BlackBox_BaroData_TypeDef));
        blackbox_ender.check_sum = check_sum;
        Monitor.baro_log.byte_size += BLACKBOX_ENDER_SIZE;
        TaskBlackBox_UpdateQueue((uint8_t *)&blackbox_ender, BLACKBOX_ENDER_SIZE);
    }
    
    if ((obj == &CtlData_Log_DataPipe) && Monitor.en_reg.bit.exp_ctl)
    {
        memset(&input_ctl_data, 0, sizeof(BlackBox_CtlData_TypeDef));
        Monitor.ctl_log.cnt ++;
        Monitor.ctl_log.byte_size += BLACKBOX_HEADER_SIZE;
        blackbox_header.type = BlackBox_CtlData;
        Monitor.ctl_log.byte_size += BLACKBOX_ENDER_SIZE;
        // TaskBlackBox_CheckQueue();
    }
    
    if ((obj == &Attitude_Log_DataPipe) && Monitor.en_reg.bit.att)
    {
        memset(&att_data, 0, sizeof(BlackBox_AttitudeData_TypeDef));
        Monitor.att_log.cnt ++;
        Monitor.att_log.byte_size += BLACKBOX_HEADER_SIZE;
        blackbox_header.type = BlackBox_Attitude;
        Monitor.att_log.byte_size += BLACKBOX_ENDER_SIZE;
        // TaskBlackBox_CheckQueue();
    }
    
    if ((obj == & Actuator_Log_DataPipe) && Monitor.en_reg.bit.act)
    {
        memset(&actuator_data, 0, sizeof(BlackBox_ActuatorData_TypeDef));
        Monitor.act_log.cnt ++;
        Monitor.act_log.byte_size += BLACKBOX_HEADER_SIZE;
        blackbox_header.type = BlackBox_Actuator;
        Monitor.act_log.byte_size += BLACKBOX_ENDER_SIZE;
        // TaskBlackBox_CheckQueue();
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
        case BlackBox_IMU:
            if (*len < (BLACKBOX_HEADER_SIZE + BLACKBOX_ENDER_SIZE + sizeof(BlackBox_IMUData_TypeDef)))
                return BlackBox_Cnv_Size_Error;
            break;

        case BlackBox_Baro:
            if (*len < (BLACKBOX_HEADER_SIZE + BLACKBOX_ENDER_SIZE + sizeof(BlackBox_BaroData_TypeDef)))
                return BlackBox_Cnv_Size_Error;
            break;

        /* still in developping */
        case BlackBox_CtlData:
        case BlackBox_Attitude:
        case BlackBox_Actuator:
            return BlackBox_Cnv_Type_Error;

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
        // shellPrint(p_shell, "[ IMU ] ");
        shellPrint(p_shell, "%d ", p_imu->time);
        shellPrint(p_shell, "%d ", p_imu->cyc);
        // shellPrint(p_shell, "%f ", p_imu->org_acc[Axis_X] / p_imu->acc_scale);
        // shellPrint(p_shell, "%f ", p_imu->org_acc[Axis_Y] / p_imu->acc_scale);
        // shellPrint(p_shell, "%f ", p_imu->org_acc[Axis_Z] / p_imu->acc_scale);
        shellPrint(p_shell, "%f ", p_imu->flt_acc[Axis_X] / p_imu->acc_scale);
        shellPrint(p_shell, "%f ", p_imu->flt_acc[Axis_Y] / p_imu->acc_scale);
        shellPrint(p_shell, "%f\r\n", p_imu->flt_acc[Axis_Z] / p_imu->acc_scale);
        // shellPrint(p_shell, "%f ", p_imu->org_gyr[Axis_X] / p_imu->gyr_scale);
        // shellPrint(p_shell, "%f ", p_imu->org_gyr[Axis_Y] / p_imu->gyr_scale);
        // shellPrint(p_shell, "%f ", p_imu->org_gyr[Axis_Z] / p_imu->gyr_scale);
        // shellPrint(p_shell, "%f ", p_imu->flt_gyr[Axis_X] / p_imu->gyr_scale);
        // shellPrint(p_shell, "%f ", p_imu->flt_gyr[Axis_Y] / p_imu->gyr_scale);
        // shellPrint(p_shell, "%f\r\n", p_imu->flt_gyr[Axis_Z] / p_imu->gyr_scale);
    }

    *p_data += sizeof(BlackBox_IMUData_TypeDef);
    *len -= sizeof(BlackBox_IMUData_TypeDef);
}

static void TaskBlackBox_ConvertLogData_To_Baro(Shell *p_shell, BlackBox_BaroData_TypeDef *p_baro, uint8_t **p_data, uint32_t *len)
{
    if ((p_baro == NULL) || \
        (p_data == NULL) || \
        (*p_data == NULL) || \
        (len == NULL) || \
        (*len < sizeof(BlackBox_BaroData_TypeDef)))
        return;

    memcpy(p_baro, *p_data, sizeof(BlackBox_BaroData_TypeDef));
    if (p_shell)
    {
        // shellPrint(p_shell, "[ Baro ] ");
        // shellPrint(p_shell, "%d ", p_baro->time);
        // shellPrint(p_shell, "%d ", p_baro->cyc);
        // shellPrint(p_shell, "%f\r\n", p_baro->press);
    }
    
    *p_data += sizeof(BlackBox_BaroData_TypeDef);
    *len -= sizeof(BlackBox_BaroData_TypeDef);
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
    BlackBox_BaroData_TypeDef log_baro;
    uint8_t *p_log_data = NULL;
    uint16_t uncomplete_size = 0;
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
                            uint16_t decode_size = 0;
                            switch ((uint8_t)header.type)
                            {
                                case BlackBox_IMU:
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

                                case BlackBox_Baro:
                                    if (uncomplete_size <= sizeof(BlackBox_BaroData_TypeDef))
                                    {
                                        memcpy(p_remain_buf + uncomplete_size, p_log_data, sizeof(BlackBox_BaroData_TypeDef) - uncomplete_size);
                                        p_log_data += sizeof(BlackBox_BaroData_TypeDef) - uncomplete_size;
                                        log_size -= sizeof(BlackBox_BaroData_TypeDef) - uncomplete_size;
                                        uncomplete_size = 0;
                                    }
                                    else
                                    {
                                        memmove(p_remain_buf, p_remain_buf + uncomplete_size, uncomplete_size - sizeof(BlackBox_IMUData_TypeDef));
                                        uncomplete_size -= sizeof(BlackBox_BaroData_TypeDef);
                                    }

                                    decode_size = sizeof(BlackBox_BaroData_TypeDef);
                                    TaskBlackBox_ConvertLogData_To_Baro(shell_obj, &log_baro, &p_remain_buf, &decode_size);
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
                                TaskBlackBox_ConvertLogData_To_Ender(shell_obj, &ender, &remain_buf, &decode_size);
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
                        case BlackBox_IMU:
                            TaskBlackBox_ConvertLogData_To_IMU(shell_obj, &log_imu, &p_log_data, &log_size);
                            check_sum = TaskBlackBox_Get_CheckSum(&log_imu, sizeof(BlackBox_IMUData_TypeDef));
                            break;

                        case BlackBox_Baro:
                            TaskBlackBox_ConvertLogData_To_Baro(shell_obj, &log_baro, &p_log_data, &log_size);
                            check_sum = TaskBlackBox_Get_CheckSum(&log_baro, sizeof(BlackBox_BaroData_TypeDef));
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
