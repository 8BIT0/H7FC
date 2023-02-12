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
    Receiver_Ctl_DataPipe.data_addr = (uint32_t)DataPipe_DataObjAddr(Control_RC_Data);
    Receiver_Ctl_DataPipe.data_size = DataPipe_DataSize(Control_RC_Data);
    Receiver_Ctl_DataPipe.trans_finish_cb = TaskControl_DataPipe_Callback;
    DataPipe_Enable(&Receiver_Ctl_DataPipe);

    DataPipe_Enable(&IMU_Ctl_DataPipe);

    TaskControl_Monitor.init_state = SrvActuator.init(DEFAULT_CONTROL_MODEL, DEFAULT_ESC_TYPE);

    if (TaskControl_Monitor.init_state)
    {
    }
}

void TaskControl_Core(Task_Handle hdl)
{
    uint16_t test_val[4] = {500, 500, 500, 500};

    if (TaskControl_Monitor.init_state)
    {
        SrvActuator.control(test_val, sizeof(test_val) / sizeof(test_val[0]));
    }
}

static void TaskControl_DataPipe_Callback(DataPipeObj_TypeDef *obj)
{

}
