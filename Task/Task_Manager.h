#ifndef __TASK_MANAGER_H
#define __TASK_MANAGER_H

#include "Task_Fusion.h"
#include "Task_Protocol.h"
#include "scheduler.h"

void Task_Manager_Init(void);
void Task_Manager_CreateTask(void);

extern Task_Handle TaskProtocol_Handle;

#endif
