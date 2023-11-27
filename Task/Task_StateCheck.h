#ifndef __TASK_STATECHECK_H
#define __TASK_STATECHECK_H

#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "Srv_OsCommon.h"
#include "Srv_DataHub.h"

void Task_StateCheck_Init(uint32_t period);
void Task_StateCheck_Core(void *arg);

#endif
