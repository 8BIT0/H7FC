#ifndef __TASK_CONTROL_H
#define __TASK_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "scheduler.h"

typedef struct
{
    bool init_state;
} TaskControl_Monitor_TypeDef;

void TaskControl_Init(void);
void TaskControl_Core(Task_Handle hdl);

#endif
