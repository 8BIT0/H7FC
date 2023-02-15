#ifndef __TASK_CONTROL_H
#define __TASK_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "scheduler.h"

typedef struct
{
    bool init_state;

    uint8_t ctl_model;

    uint32_t rc_pipe_cnt;
    uint64_t imu_pipe_cnt;

    uint16_t *ctl_buff;

} TaskControl_Monitor_TypeDef;

void TaskControl_Init(void);
void TaskControl_Core(Task_Handle hdl);

#endif
