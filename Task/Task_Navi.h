#ifndef __TASK_NAVI_H
#define __TASK_NAVI_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef struct
{
    float yaw;
    float pitch;
    float roll;
}TaskNavi_Attitude_TypeDef;

void TaskNavi_Init(uint32_t period);
void TaskNavi_Core(void const *arg);

#endif
