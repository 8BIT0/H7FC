#ifndef __TASK_TELEMETRY_H
#define __TASK_TELEMETRY_H

#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include "scheduler.h"
#include "Srv_Receiver.h"

void TaskTelemetry_Init(void);
void TaskTelemetry_Core(Task_Handle hdl);

#endif
