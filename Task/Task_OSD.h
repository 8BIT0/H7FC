#ifndef __TASK_OSD_H
#define __TASK_OSD_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "scheduler.h"

void TaskOSD_Init(void);
void TaskOSD_Core(Task_Handle hdl);

#endif

