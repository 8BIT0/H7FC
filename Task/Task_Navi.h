#ifndef __TASK_NAVI_H
#define __TASK_NAVI_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "pos_data.h"

typedef struct
{
    float att_pitch;
    float att_roll;
    float att_yaw;
    float att_heading;

    bool flip_over;

    PosData_TypeDef pos;
    PosVelData_TypeDef vel;

    uint16_t period;
}TaskNavi_Monitor_TypeDef;

void TaskNavi_Init(uint32_t period);
void TaskNavi_Core(void const *arg);

#endif
