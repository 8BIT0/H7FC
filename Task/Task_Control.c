/*
 *  coder : 8_B!T0
 *  this file use for moto & servo control
 */
#include "Task_Control.h"
#include "SrvMPU_Sample.h"
#include "Srv_Receiver.h"
#include "DataPipe.h"
#include "scheduler.h"
#include "Srv_Actuator.h"
#include "mmu.h"

#define DEFAULT_CONTROL_MODEL Model_Quad
#define DEFAULT_ESC_TYPE DevDshot_600

DataPipe_CreateDataObj(SrvIMU_UnionData_TypeDef, Filted_IMU_Data);
DataPipe_CreateDataObj(SrvRecever_RCSig_TypeDef, Control_RC_Data);

TaskControl_Monitor_TypeDef TaskControl_Monitor;

static void TaskControl_DataPipe_Callback(DataPipeObj_TypeDef *obj);

void TaskControl_Init(void)
{
    // IMU_Ctl_DataPipe
    memset(DataPipe_DataObjAddr(Control_RC_Data), 0, DataPipe_DataSize(Control_RC_Data));
    memset(DataPipe_DataObjAddr(Filted_IMU_Data), 0, DataPipe_DataSize(Filted_IMU_Data));

    Receiver_Ctl_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Control_RC_Data);
    Receiver_Ctl_DataPipe.data_size = DataPipe_DataSize(Control_RC_Data);
    Receiver_Ctl_DataPipe.trans_finish_cb = TaskControl_DataPipe_Callback;

    IMU_Ctl_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Filted_IMU_Data);
    IMU_Ctl_DataPipe.data_size = DataPipe_DataSize(Filted_IMU_Data);
    IMU_Ctl_DataPipe.trans_finish_cb = TaskControl_DataPipe_Callback;

    DataPipe_Set_RxInterval(&IMU_Ctl_DataPipe, 1000);

    DataPipe_Enable(&Receiver_Ctl_DataPipe);
    DataPipe_Enable(&IMU_Ctl_DataPipe);

    TaskControl_Monitor.init_state = SrvActuator.init(DEFAULT_CONTROL_MODEL, DEFAULT_ESC_TYPE);

    if (TaskControl_Monitor.init_state)
    {
        TaskControl_Monitor.actuator_num = SrvActuator.get_cnt().total_cnt;
        TaskControl_Monitor.ctl_buff = (uint16_t *)MMU_Malloc(sizeof(uint16_t) * TaskControl_Monitor.actuator_num);

        if (TaskControl_Monitor.ctl_buff == NULL)
        {
            MMU_Free(TaskControl_Monitor.ctl_buff);
            TaskControl_Monitor.init_state = false;
            return;
        }

        memset(TaskControl_Monitor.ctl_buff, 0, sizeof(uint16_t) * TaskControl_Monitor.actuator_num);
    }
}

void TaskControl_Core(Task_Handle hdl)
{
    if (TaskControl_Monitor.init_state)
    {
        /* only manipulate esc or servo when disarm */
        if (!DataPipe_DataObj(Control_RC_Data).arm_state)
        {
            SrvActuator.control(TaskControl_Monitor.ctl_buff, TaskControl_Monitor.actuator_num);
        }
        else
            SrvActuator.lock();
    }
}

static void TaskControl_DataPipe_Callback(DataPipeObj_TypeDef *obj)
{
    static uint8_t r_cnt;
    static uint8_t i_cnt;

    if (obj == NULL)
        return;

    if (obj == &Receiver_Ctl_DataPipe)
    {
        r_cnt++;
    }
    else if (obj == &IMU_Ctl_DataPipe)
    {
        i_cnt++;
    }
}
