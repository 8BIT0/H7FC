/*
 *  coder : 8_B!T0
 *  this file use for moto & servo control
 */
#include "Task_Control.h"
#include "DataPipe.h"
#include "scheduler.h"
#include "Srv_Actuator.h"
#include "mmu.h"

#define DEFAULT_CONTROL_MODEL Model_Quad
#define DEFAULT_ESC_TYPE DevDshot_600

TaskControl_Monitor_TypeDef TaskControl_Monitor;

void TaskControl_Init(void)
{
    TaskControl_Monitor.init_state = SrvActuator.init(DEFAULT_CONTROL_MODEL, DEFAULT_ESC_TYPE);

    if (TaskControl_Monitor.init_state)
    {
    }
}

void TaskControl_Core(Task_Handle hdl)
{
    uint16_t test_val[4] = {200, 200, 200, 200};

    if(TaskControl_Monitor.init_state)
    {
        SrvActuator.control(test_val, sizeof(test_val) / sizeof(test_val[0]));
    }
}
