#ifndef __TASK_CONTROL_H
#define __TASK_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "scheduler.h"

#define IMU_ERROR_UPDATE_MAX_COUNT 10

typedef struct
{
    bool init_state;
    bool control_abort;

    uint8_t ctl_model;
    uint8_t actuator_num;

    uint32_t rc_pipe_cnt;
    uint64_t imu_pipe_cnt;

    uint16_t *ctl_buff;

    uint8_t imu_update_error_cnt;
} TaskControl_Monitor_TypeDef;

void TaskControl_Init(void);
void TaskControl_Core(Task_Handle hdl);

#endif
