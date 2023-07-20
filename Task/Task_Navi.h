#ifndef __TASK_NAVI_H
#define __TASK_NAVI_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

void TaskNavi_Init(uint32_t period);
void TaskNavi_Core(void const *arg);

#endif
